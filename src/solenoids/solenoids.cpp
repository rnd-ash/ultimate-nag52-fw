#include "solenoids.h"
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

OnOffSolenoid *sol_y3 = nullptr;
OnOffSolenoid *sol_y4 = nullptr;
OnOffSolenoid *sol_y5 = nullptr;

ConstantCurrentSolenoid *sol_mpc = nullptr;
ConstantCurrentSolenoid *sol_spc = nullptr;
InrushControlSolenoid *sol_tcc = nullptr;

#define NUM_SOLENOIDS 6
struct SolenoidOutputSummary {
    uint64_t peak_total[NUM_SOLENOIDS];
    uint16_t count_peak[NUM_SOLENOIDS];
    uint16_t count_total[NUM_SOLENOIDS];
};

QueueHandle_t solenoid_summery_queue;

/*
6 channels
each channel:
    200 samples per 'spike' (1000hz)
    record 2 spikes, and get average
*/
#define I2S_DMA_BUF_LEN 6 * 200 * SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2
uint8_t adc_read_buf[I2S_DMA_BUF_LEN];
bool first_read_complete = false;
uint64_t isr_done = 0;
uint8_t CHANNEL_ID_MAP[ADC_CHANNEL_9] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static bool IRAM_ATTR on_i2s_read(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    SolenoidOutputSummary s = {
        .peak_total = {0,0,0,0,0,0},
        .count_peak = {0,0,0,0,0,0},
        .count_total = {0,0,0,0,0,0},
    };
    for (int i = 0; i < edata->size; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&edata->conv_frame_buffer[i];
        uint8_t channel_idx = CHANNEL_ID_MAP[p->type1.channel];
        if (channel_idx != 0xFF) {
            if (p->type1.data > 120) { // > ~0.1V
                s.peak_total[channel_idx] += p->type1.data;
                s.count_peak[channel_idx] += 1;
            }
            s.count_total[channel_idx] += 1;
        }
    }
    BaseType_t taskWoken = pdFALSE;
    isr_done = esp_timer_get_time();
    xQueueSendFromISR(solenoid_summery_queue, &s, &taskWoken);
    return (taskWoken == pdTRUE);
}

void read_solenoids_i2s(void*) {
    PwmSolenoid* const sol_order[6]  = { sol_mpc, sol_spc, sol_y3, sol_y4, sol_y5, sol_tcc };
    adc_continuous_handle_t c_handle = nullptr;
    const adc_continuous_handle_cfg_t c_cfg = {
        .max_store_buf_size = I2S_DMA_BUF_LEN*2,
        .conv_frame_size = I2S_DMA_BUF_LEN,
    };
    adc_continuous_new_handle(&c_cfg, &c_handle);
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    for (int i = 0; i < NUM_SOLENOIDS; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = sol_order[i]->get_adc_channel() & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; // 12bits
        CHANNEL_ID_MAP[(uint8_t)sol_order[i]->get_adc_channel()] = i;
    }
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 6, 
        .adc_pattern = adc_pattern,
        .sample_freq_hz = 732000*2, // Real freq is 600000hz. (Bug with IDF 5.1) 2000000
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    adc_continuous_config(c_handle, &dig_cfg);

    const adc_continuous_evt_cbs_t callbacks = {
        .on_conv_done = on_i2s_read,
        .on_pool_ovf = nullptr
    };

    adc_continuous_register_event_callbacks(c_handle, &callbacks, nullptr);
    solenoid_summery_queue = xQueueCreate(4, sizeof(SolenoidOutputSummary));
    adc_continuous_start(c_handle);
    SolenoidOutputSummary s;
    while(true) {
        xQueueReceive(solenoid_summery_queue, &s, portMAX_DELAY);
        for (int i = 0; i < 6; i++) {
            if (s.count_peak[i] > 0) {
                sol_order[i]->__set_adc_reading((float)s.peak_total[i]/(float)s.count_peak[i]);
            } else {
                sol_order[i]->__set_adc_reading(0);
            }
            // Yields 800 samples/sol/4ms
            // Each solenoid is being sampled at 200 samples per wave
            // Important step for CC solenoids
            if (sol_order[i] == sol_mpc) {
                sol_mpc->update_when_reading(voltage);
            } else if (sol_order[i] == sol_spc) {
                //printf("%d %d\n", (int)s.count_peak[i], (int)s.count_total[i]);
                sol_spc->update_when_reading(voltage);
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
    uint16_t vbatt = TCUIO::battery_mv();
    int16_t atf = TCUIO::atf_temperature();
    while(true) {
        vbatt = TCUIO::battery_mv();
        atf = TCUIO::atf_temperature();
        if (UINT16_MAX != vbatt) {
            voltage = vbatt;
            vref_compensation = (float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)voltage;
        } else {
            vref_compensation = 1.0;
        }
        if (INT16_MAX != atf) {
            atf_temp = atf*10.0;
            temp_compensation = (((atf_temp-(SOL_CURRENT_SETTINGS.cc_reference_temp*10.0))/10.0)*SOL_CURRENT_SETTINGS.cc_temp_coefficient_wires)/10.0;
        }
        if (write_pwm) {
            // MOVED TO CURRENT READING TASK SO READINGS ARE SYNCED
            //sol_mpc->__write_pwm(vref_compensation, temp_compensation, vbatt_too_low);
            //sol_spc->__write_pwm(vref_compensation, temp_compensation, vbatt_too_low);
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
    sol_y3 = new OnOffSolenoid("Y3", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y3_pwm, ledc_channel_t::LEDC_CHANNEL_0, ADC_CHANNEL_0, 100, 1524, 1);
    sol_y4 = new OnOffSolenoid("Y4", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y4_pwm, ledc_channel_t::LEDC_CHANNEL_1, ADC_CHANNEL_3, 100, 1524, 1);
    sol_y5 = new OnOffSolenoid("Y5", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y5_pwm, ledc_channel_t::LEDC_CHANNEL_2, ADC_CHANNEL_7, 100, 1524, 1);
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