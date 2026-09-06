#include "solenoids.h"
#include <atomic>
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "board_config.h"
#include "../sensors.h"
#include "soc/i2s_periph.h"
#include "string.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "../nvs/module_settings.h"
#include "clock.hpp"
#include "esp_timer.h"
#include "tcu_io/tcu_io.hpp"

OnOffSolenoid* sol_y3 = nullptr;
OnOffSolenoid* sol_y4 = nullptr;
OnOffSolenoid* sol_y5 = nullptr;

ConstantCurrentSolenoid* sol_mpc = nullptr;
ConstantCurrentSolenoid* sol_spc = nullptr;
InrushControlSolenoid* sol_tcc = nullptr;

#define NUM_SOLENOIDS (6)
struct SolenoidOutputSummary {
    uint64_t peak_total[NUM_SOLENOIDS];
    uint16_t count_peak[NUM_SOLENOIDS];
};

/*
6 channels
each channel:
    200 samples per 'spike' (1000hz)
    record 2 spikes, and get average
    ~2400000sps = 400000sps per solenoid
*/
#define I2S_DMA_BUF_LEN ((6) * (200) * (SOC_ADC_DIGI_DATA_BYTES_PER_CONV) * (2))
uint8_t adc_read_buf[I2S_DMA_BUF_LEN];
// Task-to-task flags with no shared lock. Relaxed: each is an independent
// flag that gates only itself, so no acquire/release pairing is needed.
std::atomic<bool> first_read_complete{false};
uint64_t isr_done = 0;
// Indexed by the 4 bit channel field of the ADC DMA result, so it has to cover
// the full 0-15 range that field can hold, not just the channels we configured.
#define ADC_CHANNEL_MAP_SIZE (16)
uint8_t CHANNEL_ID_MAP[ADC_CHANNEL_MAP_SIZE] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

void read_solenoids_i2s(void*) {
    PwmSolenoid* const sol_order[6] = { sol_mpc, sol_spc, sol_y3, sol_y4, sol_y5, sol_tcc };
    adc_continuous_handle_t c_handle = nullptr;
    adc_continuous_handle_cfg_t c_cfg = {};
    c_cfg.max_store_buf_size = I2S_DMA_BUF_LEN * 2;
    c_cfg.conv_frame_size = I2S_DMA_BUF_LEN;
    esp_err_t setup_res = adc_continuous_new_handle(&c_cfg, &c_handle);
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {};
    for (int i = 0; i < NUM_SOLENOIDS; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = sol_order[i]->get_adc_channel() & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; // 12bits
        uint8_t chan = (uint8_t)sol_order[i]->get_adc_channel();
        if (chan < ADC_CHANNEL_MAP_SIZE) {
            CHANNEL_ID_MAP[chan] = i;
        }
    }
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 6,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = 732000 * 2, // Real freq is 600000hz. (Bug with IDF 5.1) 2000000
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    if (ESP_OK == setup_res) {
        setup_res = adc_continuous_config(c_handle, &dig_cfg);
    }
    if (ESP_OK == setup_res) {
        setup_res = adc_continuous_start(c_handle);
    }
    if (ESP_OK != setup_res) {
        // Without this the read below blocks forever on portMAX_DELAY, and
        // current sensing silently never comes up.
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Could not start solenoid current sensing: %s", esp_err_to_name(setup_res));
        vTaskDelete(NULL);
        return;
    }
    esp_err_t ret;
    uint32_t read_len;
    while (true) {
        // Process solenoid info
        SolenoidOutputSummary s = {
            .peak_total = {0,0,0,0,0,0},
            .count_peak = {0,0,0,0,0,0},
        };
        // Runs every 4ms or so
        ret = adc_continuous_read(c_handle, adc_read_buf, I2S_DMA_BUF_LEN, &read_len, portMAX_DELAY);
        if (ESP_OK == ret) {
            for (uint32_t i = 0; i < read_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
                // adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_read_buf[i];
                adc_digi_output_data_t* p = reinterpret_cast<adc_digi_output_data_t*>(&adc_read_buf[i]);
                uint8_t channel_idx = CHANNEL_ID_MAP[p->type1.channel];
                if (channel_idx != 0xFF) {
                    if (p->type1.data != 0) {
                        s.peak_total[channel_idx] += p->type1.data;
                        s.count_peak[channel_idx] += 1;
                    }
                }
            }

            for (int i = 0; i < 6; i++) {
                if (s.count_peak[i] > 0) {
                    sol_order[i]->__set_adc_reading((float)s.peak_total[i] / (float)s.count_peak[i]);
                }
                else {
                    sol_order[i]->__set_adc_reading(0);
                }
            }
            sol_mpc->update_when_reading(voltage);
            sol_spc->update_when_reading(voltage);
            if (!first_read_complete.load(std::memory_order_relaxed)) {
                first_read_complete.store(true, std::memory_order_relaxed);
            }
        }
    }
}

std::atomic<bool> write_pwm{true};
static portMUX_TYPE cal_data_lock = portMUX_INITIALIZER_UNLOCKED;

void Solenoids::notify_diag_test_start() {
    // Reachable from diag before/without a successful solenoid init
    if (nullptr != sol_mpc) { sol_mpc->set_current_target(0); }
    if (nullptr != sol_spc) { sol_spc->set_current_target(0); }
    if (nullptr != sol_tcc) { sol_tcc->set_duty(0); }
    if (nullptr != sol_y3) { sol_y3->off(); }
    if (nullptr != sol_y4) { sol_y4->off(); }
    if (nullptr != sol_y5) { sol_y5->off(); }
    vTaskDelay(20);
    write_pwm.store(false, std::memory_order_relaxed);
}

void Solenoids::notify_diag_test_end(void) {
    write_pwm.store(true, std::memory_order_relaxed);
}

void update_solenoids(void*) {
    while (true) {
        float vref_compensation = 1.0F;
        float temp_compensation = 1.0F;
        uint16_t vbatt = TCUIO::battery_mv();
        temp_c_t atf = TCUIO::atf_temperature();
        if (UINT16_MAX != vbatt) {
            voltage = vbatt;
            vref_compensation = (float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)voltage;
        }
        else {
            vref_compensation = 1.0F;
        }
        if (Temp::is_valid(atf)) {
            float atf_temp = (float)Temp::celsius_i16(atf) * 10.0F;
            temp_compensation = (((atf_temp - (SOL_CURRENT_SETTINGS.cc_reference_temp * 10.0F)) / 10.0F) * SOL_CURRENT_SETTINGS.cc_temp_coefficient_wires) / 10.0F;
        }
        if (write_pwm.load(std::memory_order_relaxed)) {
            // MOVED TO CURRENT READING TASK SO READINGS ARE SYNCED
            sol_tcc->__write_pwm(vref_compensation, temp_compensation);
            sol_y3->__write_pwm(vref_compensation, temp_compensation);
            sol_y4->__write_pwm(vref_compensation, temp_compensation);
            sol_y5->__write_pwm(vref_compensation, temp_compensation);
        }
        vTaskDelay(1); // Max we can do at 1000hz
    }
}

float resistance_mpc = 5.0f;
float resistance_spc = 5.0f;
bool temp_cal = false;
int16_t temp_at_test = 25;

std::atomic<bool> routine{false};
std::atomic<bool> startup_ok{false};


gpio_num_t Solenoids::get_tcc_zener_pin(void) {
    // Gated on "does this board actually map an IO_0 pin", NOT on
    // BOARD_CONFIG.board_ver == 3.
    //
    // Officially the TCC zener board is a 1.3 addon (and is integrated on
    // 1.3 Rev B), and BoardV13GpioMatrix is the only matrix that maps io_pin
    // today - so this is behaviourally identical to the old board_ver check.
    // The difference is that hand-wiring the mod onto an older board becomes a
    // one line change in board_config.cpp (assign that revision an io_pin)
    // rather than an edit here as well. Boards with no IO_0 pin return
    // GPIO_NUM_NC and fall back to the original PWM-only TCC control.
    gpio_num_t ret = GPIO_NUM_NC;
    if (nullptr != pcb_gpio_matrix && IO_0_USAGE_TCC_ZENER == VEHICLE_CONFIG.io_0_usage) {
        ret = pcb_gpio_matrix->io_pin;
    }
    return ret;
}

uint16_t Solenoids::get_solenoid_voltage(void) {
    return voltage;
}

bool Solenoids::init_routine_completed(void) {
    return routine.load(std::memory_order_relaxed);
}

void Solenoids::set_calibration_adjusted_resistance(float new_spc, float new_mpc, int16_t temp_c) {
    portENTER_CRITICAL(&cal_data_lock);
    resistance_spc = new_spc;
    resistance_mpc = new_mpc;
    temp_at_test = temp_c;
    temp_cal = true;
    portEXIT_CRITICAL(&cal_data_lock);
}

void Solenoids::get_calibration_adjusted_resistance(float* out_spc, float* out_mpc, bool* out_calibrated, int16_t* out_temp_c) {
    portENTER_CRITICAL(&cal_data_lock);
    if (out_spc != nullptr) {
        *out_spc = resistance_spc;
    }
    if (out_mpc != nullptr) {
        *out_mpc = resistance_mpc;
    }
    if (out_calibrated != nullptr) {
        *out_calibrated = temp_cal;
    }
    if (out_temp_c != nullptr) {
        *out_temp_c = temp_at_test;
    }
    portEXIT_CRITICAL(&cal_data_lock);
}

bool Solenoids::startup_test_ok(void) {
    return startup_ok.load(std::memory_order_relaxed);
}

bool Solenoids::wait_for_boot_test(uint32_t timeout_ms) {
    uint32_t start = GET_CLOCK_TIME();
    while (!Solenoids::init_routine_completed() && (GET_CLOCK_TIME() - start) < timeout_ms) {
        vTaskDelay(1);
    }
    return Solenoids::init_routine_completed() && Solenoids::startup_test_ok();
}

esp_err_t Solenoids::start_boot_test(void) {
    routine.store(false, std::memory_order_relaxed);
    startup_ok.store(false, std::memory_order_relaxed);
    if (xTaskCreate(Solenoids::boot_solenoid_test, "SOL-BOOT", 4096, nullptr, 4, nullptr) != pdPASS) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Could not create the solenoid boot test task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void Solenoids::boot_solenoid_test(void*) {
    // In zener mode the TCC solenoid is held off by the MOSFET rather than by a
    // zero PWM duty, so its "current when off" reading is not meaningful here.
    bool tcc_new_mode = (GPIO_NUM_NC != Solenoids::get_tcc_zener_pin());
    // Bounded wait. If current sensing failed to start we must not spin here
    // forever, or init_routine_completed() never returns true and boot stalls.
    uint32_t wait_start = GET_CLOCK_TIME();
    while (!first_read_complete.load(std::memory_order_relaxed)) {
        if ((GET_CLOCK_TIME() - wait_start) > SOLENOID_FIRST_READ_TIMEOUT_MS) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Timed out waiting for the first current reading!");
            startup_ok.store(false, std::memory_order_relaxed);
            routine.store(true, std::memory_order_relaxed);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(1);
    }
    if (sol_spc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "SPC drawing too much current when off!");
        routine.store(true, std::memory_order_relaxed);
        startup_ok.store(false, std::memory_order_relaxed);
        return;
    }

    if (sol_mpc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "MPC drawing too much current when off!");
        routine.store(true, std::memory_order_relaxed);
        startup_ok.store(false, std::memory_order_relaxed);
        return;
    }

    if (!tcc_new_mode && sol_tcc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "TCC drawing too much current when off!");
        routine.store(true, std::memory_order_relaxed);
        startup_ok.store(false, std::memory_order_relaxed);
        return;
    }

    if (sol_y3->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y3 drawing too much current when off!");
        routine.store(true, std::memory_order_relaxed);
        startup_ok.store(false, std::memory_order_relaxed);
        return;
    }

    if (sol_y4->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y4 drawing too much current when off!");
        routine.store(true, std::memory_order_relaxed);
        startup_ok.store(false, std::memory_order_relaxed);
        return;
    }

    if (sol_y5->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y5 drawing too much current when off!");
        routine.store(true, std::memory_order_relaxed);
        startup_ok.store(false, std::memory_order_relaxed);
        return;
    }
    startup_ok.store(true, std::memory_order_relaxed);
    routine.store(true, std::memory_order_relaxed);
    vTaskDelete(NULL);
}

esp_err_t Solenoids::init_all_solenoids()
{
    SolenoidSetup::init_adc();
    // Read calibration for ADC1
    sol_y3 = new OnOffSolenoid("Y3", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y3_pwm, ledc_channel_t::LEDC_CHANNEL_0, ADC_CHANNEL_0, 250, 1524, 1);
    sol_y4 = new OnOffSolenoid("Y4", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y4_pwm, ledc_channel_t::LEDC_CHANNEL_1, ADC_CHANNEL_3, 250, 1524, 1);
    sol_y5 = new OnOffSolenoid("Y5", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y5_pwm, ledc_channel_t::LEDC_CHANNEL_2, ADC_CHANNEL_7, 250, 1524, 1);
    sol_mpc = new ConstantCurrentSolenoid("MPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->mpc_pwm, ledc_channel_t::LEDC_CHANNEL_3, ADC_CHANNEL_6, 1);
    sol_spc = new ConstantCurrentSolenoid("SPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->spc_pwm, ledc_channel_t::LEDC_CHANNEL_4, ADC_CHANNEL_4, 1);

    // ~700mA for TCC solenoid when holding
    const gpio_num_t gpio_zener = Solenoids::get_tcc_zener_pin();
    sol_tcc = new InrushControlSolenoid("TCC", ledc_timer_t::LEDC_TIMER_1, pcb_gpio_matrix->tcc_pwm, gpio_zener, ledc_channel_t::LEDC_CHANNEL_5, ADC_CHANNEL_5, 100, 700, 10);
    ESP_RETURN_ON_ERROR(sol_tcc->init_ok(), "SOLENOID", "TCC init not OK");
    ESP_RETURN_ON_ERROR(sol_mpc->init_ok(), "SOLENOID", "MPC init not OK");
    ESP_RETURN_ON_ERROR(sol_spc->init_ok(), "SOLENOID", "SPC init not OK");
    ESP_RETURN_ON_ERROR(sol_y3->init_ok(), "SOLENOID", "Y3 init not OK");
    ESP_RETURN_ON_ERROR(sol_y4->init_ok(), "SOLENOID", "Y4 init not OK");
    ESP_RETURN_ON_ERROR(sol_y5->init_ok(), "SOLENOID", "Y5 init not OK");
    xTaskCreate(update_solenoids, "LEDC-Update", 8192, nullptr, 10, nullptr);
    xTaskCreatePinnedToCore(read_solenoids_i2s, "I2S-Reader", 2048, nullptr, 3, nullptr, 1);
    // NOTE: the power on test is NOT started here. It is threshold sensitive, so
    // it has to wait until the module settings are loaded - see start_boot_test().
    return ESP_OK;
}