#include "solenoids.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "board_config.h"
#include "../sensors.h"
#include "soc/syscon_periph.h"
#include "soc/i2s_periph.h"
#include "string.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "../nvs/module_settings.h"
#include "clock.hpp"

OnOffSolenoid *sol_y3 = nullptr;
OnOffSolenoid *sol_y4 = nullptr;
OnOffSolenoid *sol_y5 = nullptr;

ConstantCurrentSolenoid *sol_mpc = nullptr;
ConstantCurrentSolenoid *sol_spc = nullptr;
InrushControlSolenoid *sol_tcc = nullptr;

// 6 channels * SOC_ADC_DIGI_DATA_BYTES_PER_CONV bytes per sample *  
#define I2S_DMA_BUF_LEN 6 * 100 * SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 4
uint8_t adc_read_buf[I2S_DMA_BUF_LEN];
bool first_read_complete = false;

void read_solenoids_i2s(void*) {
    PwmSolenoid* const sol_order[6]  = { sol_mpc, sol_spc, sol_y3, sol_y4, sol_y5, sol_tcc };
    adc_continuous_handle_t c_handle = nullptr;
    const adc_continuous_handle_cfg_t c_cfg = {
        .max_store_buf_size = I2S_DMA_BUF_LEN*4,
        .conv_frame_size = I2S_DMA_BUF_LEN,
    };
    adc_continuous_new_handle(&c_cfg, &c_handle);
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 6, 
        .sample_freq_hz = 732000, // Real freq is 600000hz. (Bug with IDF 5.1) 2000000
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    for (int i = 0; i < 6; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = sol_order[i]->get_adc_channel() & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; // 12bits
    }
    dig_cfg.adc_pattern = adc_pattern;
    adc_continuous_config(c_handle, &dig_cfg);
    adc_continuous_start(c_handle);

    esp_err_t ret;

    PwmSolenoid* sol_order_by_adc_channel[adc_channel_t::ADC_CHANNEL_9] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    uint64_t totals[ADC_BITWIDTH_9];
    uint16_t peak_samples[ADC_BITWIDTH_9];
    uint16_t total_count[ADC_CHANNEL_9];
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t idx = sol_order[i]->get_adc_channel();
        sol_order_by_adc_channel[idx] = sol_order[i];
        total_count[idx] = 0;
        peak_samples[idx] = 0;
        totals[idx] = 0;
    }
    uint32_t out_len = 0;
    while(true) {
        ret = adc_continuous_read(c_handle, adc_read_buf, I2S_DMA_BUF_LEN, &out_len, portMAX_DELAY);
        if (ret == ESP_OK) {
            for (int i = 0; i < out_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_read_buf[i];
                uint8_t channel_idx = p->type1.channel;
                if (p->type1.data != 0) {
                    peak_samples[channel_idx] += 1;
                    totals[channel_idx] += p->type1.data;
                }
                total_count[channel_idx] += 1;
                if (total_count[channel_idx] == 200*sol_order_by_adc_channel[channel_idx]->get_pwm_phase_time()) { // over 4ms
                    if (peak_samples[channel_idx] >= sol_order_by_adc_channel[channel_idx]->get_pwm_phase_time()) {
                        sol_order_by_adc_channel[channel_idx]->__set_adc_reading((float)totals[channel_idx]/(float)peak_samples[channel_idx]);
                    } else {
                        sol_order_by_adc_channel[channel_idx]->__set_adc_reading(0);
                    }

                    // Important step for CC solenoids
                    if (sol_order_by_adc_channel[channel_idx] == sol_mpc) {
                        sol_mpc->set_target_current_when_reading();
                    } else if (sol_order_by_adc_channel[channel_idx] == sol_spc) {
                        sol_spc->set_target_current_when_reading();
                    }

                    totals[channel_idx] = 0;
                    peak_samples[channel_idx] = 0;
                    total_count[channel_idx] = 0;
                }
            }
        }
    }
}

bool write_pwm = true;

void Solenoids::notify_diag_test_start() {
    sol_mpc->set_current_target(0);
    sol_spc->set_current_target(0);
    sol_tcc->set_duty(0);
    sol_y3->off();
    sol_y4->off();
    sol_y5->off();
    vTaskDelay(20);
    write_pwm = false;
}

void Solenoids::notify_diag_test_end(void) {
    write_pwm = true;
}

void update_solenoids(void*) {
    int16_t atf_temp = 250;
    float vref_compensation = 1.0;
    float temp_compensation = 1.0;
    while(true) {
        if (ESP_OK == Sensors::read_vbatt(&voltage)) {
            vref_compensation = (float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)voltage;
        } else {
            vref_compensation = 1.0;
        }
        // Only query ATF temp when not changing gears
        bool ss_full_on = sol_y3->is_max_on() || sol_y4->is_max_on() || sol_y5->is_max_on();
        if (!ss_full_on && ESP_OK == Sensors::read_atf_temp_fine(&atf_temp)) {
            temp_compensation = (((atf_temp-(SOL_CURRENT_SETTINGS.cc_reference_temp*10.0))/10.0)*SOL_CURRENT_SETTINGS.cc_temp_coefficient_wires)/10.0;
        }
        if (write_pwm) {
            sol_mpc->__write_pwm(vref_compensation, temp_compensation, ss_full_on);
            sol_spc->__write_pwm(vref_compensation, temp_compensation, ss_full_on);
            sol_tcc->__write_pwm(vref_compensation, temp_compensation);
            sol_y3->__write_pwm(vref_compensation, temp_compensation);
            sol_y4->__write_pwm(vref_compensation, temp_compensation);
            sol_y5->__write_pwm(vref_compensation, temp_compensation);
        }
        vTaskDelay(1); // Max we can do at 1000hz
    }
}

float resistance_mpc = 5.0;
float resistance_spc = 5.0;
bool temp_cal = false;
int16_t temp_at_test = 25;

bool routine = false;
bool startup_ok = false;


uint16_t Solenoids::get_solenoid_voltage(void) {
    return voltage;
}

bool Solenoids::init_routine_completed(void) {
    return routine;
}

// bool Solenoids::startup_test_ok() {
//     return startup_ok;
// }

void Solenoids::boot_solenoid_test(void*) {
    while(!first_read_complete){vTaskDelay(1);}
    if(sol_spc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "SPC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_mpc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "MPC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_tcc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "TCC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_y3->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y3 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_y4->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y4 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_y5->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y5 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }
    startup_ok = true;
    routine = true;
    vTaskDelete(NULL);
}

esp_err_t Solenoids::init_all_solenoids()
{
    SolenoidSetup::init_adc();
    // Read calibration for ADC1
    sol_y3 = new OnOffSolenoid("Y3", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y3_pwm, ledc_channel_t::LEDC_CHANNEL_0, ADC_CHANNEL_0, 250, 1524, 1);
    sol_y4 = new OnOffSolenoid("Y4", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y4_pwm, ledc_channel_t::LEDC_CHANNEL_1, ADC_CHANNEL_3, 250, 1524, 1);
    sol_y5 = new OnOffSolenoid("Y5", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y5_pwm, ledc_channel_t::LEDC_CHANNEL_2, ADC_CHANNEL_7, 250, 1524, 1);
    sol_mpc = new ConstantCurrentSolenoid("MPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->mpc_pwm, ledc_channel_t::LEDC_CHANNEL_3, ADC_CHANNEL_6, 1, true); 
    sol_spc = new ConstantCurrentSolenoid("SPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->spc_pwm, ledc_channel_t::LEDC_CHANNEL_4, ADC_CHANNEL_4, 1, false);
    // ~700mA for TCC solenoid when holding
    sol_tcc = new InrushControlSolenoid("TCC", ledc_timer_t::LEDC_TIMER_1, pcb_gpio_matrix->tcc_pwm, ledc_channel_t::LEDC_CHANNEL_5, ADC_CHANNEL_5, 100, 700, 10);
    ESP_RETURN_ON_ERROR(sol_tcc->init_ok(), "SOLENOID", "TCC init not OK");
    ESP_RETURN_ON_ERROR(sol_mpc->init_ok(), "SOLENOID", "MPC init not OK");
    ESP_RETURN_ON_ERROR(sol_spc->init_ok(), "SOLENOID", "SPC init not OK");
    ESP_RETURN_ON_ERROR(sol_y3->init_ok(), "SOLENOID", "Y3 init not OK");
    ESP_RETURN_ON_ERROR(sol_y4->init_ok(), "SOLENOID", "Y4 init not OK");
    ESP_RETURN_ON_ERROR(sol_y5->init_ok(), "SOLENOID", "Y5 init not OK");
    xTaskCreate(update_solenoids, "LEDC-Update", 8192, nullptr, 10, nullptr);
    xTaskCreatePinnedToCore(read_solenoids_i2s, "I2S-Reader", 2048, nullptr, 3, nullptr, 1);
    //xTaskCreate(Solenoids::boot_solenoid_test, "Solenoid-Test", 8192, nullptr, 3, nullptr);
    return ESP_OK;
}