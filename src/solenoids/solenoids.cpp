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

OnOffSolenoid *sol_y3 = nullptr;
OnOffSolenoid *sol_y4 = nullptr;
OnOffSolenoid *sol_y5 = nullptr;

adc_cali_handle_t adc1_cal = nullptr;

ConstantCurrentSolenoid *sol_mpc = nullptr;
ConstantCurrentSolenoid *sol_spc = nullptr;
InrushControlSolenoid *sol_tcc = nullptr;

#define I2S_DMA_BUF_LEN 2048
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
        .sample_freq_hz = 1000 * 1000, // Each read is ~2ms of data
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
    uint32_t samples[adc_channel_t::ADC_CHANNEL_9]; // Indexes all ADC channels like this
    uint64_t totals[adc_channel_t::ADC_CHANNEL_9];
    uint32_t out_len = 0;
    uint32_t channel_num = 0;
    uint8_t idx = 0;
    int v = 0;
    while(true) {
        memset(samples, 0, sizeof(samples));
        memset(totals, 0, sizeof(totals));
        ret = adc_continuous_read(c_handle, adc_read_buf, I2S_DMA_BUF_LEN, &out_len, portMAX_DELAY);
        if (ret == ESP_OK) {
            for (int i = 0; i < out_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_read_buf[i];
                channel_num = p->type1.channel;
                if (channel_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1) && p->type1.data > 10) {
                    totals[channel_num] += p->type1.data;
                    samples[channel_num]++;
                }
            }
            for (int solenoid = 0; solenoid < 6; solenoid++) {
                idx = (uint8_t)sol_order[solenoid]->get_adc_channel(); // Channel index
                adc_cali_raw_to_voltage(adc1_cal, totals[idx] / (float)samples[idx], &v);
                sol_order[solenoid]->__set_adc_reading((samples[idx] == 0) ? 0 :  v);
            }
            first_read_complete = true;
        }
    }
}



void update_solenoids(void*) {
    int16_t atf_temp = 25;
    float vref_compensation = 1.0;
    float temp_compensation = 1.0;
    while(true) {
        if (ESP_OK == Sensors::read_vbatt(&voltage)) {
            vref_compensation = (float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)voltage;
        } else {
            vref_compensation = 1.0;
        }
        if (ESP_OK == Sensors::read_atf_temp(&atf_temp)) {
            temp_compensation = ((atf_temp-SOL_CURRENT_SETTINGS.cc_reference_temp)*SOL_CURRENT_SETTINGS.cc_temp_coefficient_wires)/100.0;
        } else {
            vref_compensation = 1.0;
        }

        sol_mpc->__write_pwm(vref_compensation, temp_compensation);
        sol_spc->__write_pwm(vref_compensation, temp_compensation);
        sol_tcc->__write_pwm(vref_compensation, temp_compensation);
        sol_y3->__write_pwm(vref_compensation, temp_compensation);
        sol_y4->__write_pwm(vref_compensation, temp_compensation);
        sol_y5->__write_pwm(vref_compensation, temp_compensation);

        vTaskDelay(1); // Max we can do at 1000hz
    }
}

float resistance_mpc = 5.0;
float resistance_spc = 5.0;
bool temp_cal = false;
int16_t temp_at_test = 25;

bool routine = false;
bool startup_ok = false;


uint16_t Solenoids::get_solenoid_voltage() {
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
    const adc_cali_line_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .atten = adc_atten_t::ADC_ATTEN_DB_11,
        .bitwidth = adc_bitwidth_t::ADC_BITWIDTH_12,
        .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_line_fitting(&cali, &adc1_cal), "SOLENOID", "Failed to create ADC cal");
    // Set the minimum ADC Reading in mV
    ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(adc1_cal, 0, (int*)&min_adc_v_reading), "SOLENOID", "Failed to get ADC min value");
    // Read calibration for ADC1
    sol_y3 = new OnOffSolenoid("Y3", pcb_gpio_matrix->y3_pwm, ledc_channel_t::LEDC_CHANNEL_0, ADC_CHANNEL_0, 500, 500, 5);
    sol_y4 = new OnOffSolenoid("Y4", pcb_gpio_matrix->y4_pwm, ledc_channel_t::LEDC_CHANNEL_1, ADC_CHANNEL_3, 500, 500, 5);
    sol_y5 = new OnOffSolenoid("Y5", pcb_gpio_matrix->y5_pwm, ledc_channel_t::LEDC_CHANNEL_2, ADC_CHANNEL_7, 500, 500, 5);
    sol_mpc = new ConstantCurrentSolenoid("MPC", pcb_gpio_matrix->mpc_pwm, ledc_channel_t::LEDC_CHANNEL_3, ADC_CHANNEL_6, 5); 
    sol_spc = new ConstantCurrentSolenoid("SPC", pcb_gpio_matrix->spc_pwm, ledc_channel_t::LEDC_CHANNEL_4, ADC_CHANNEL_4, 5);
    // ~700mA for TCC solenoid when holding
    sol_tcc = new InrushControlSolenoid("TCC", pcb_gpio_matrix->tcc_pwm, ledc_channel_t::LEDC_CHANNEL_5, ADC_CHANNEL_5, 50, 700, 20);
    ESP_RETURN_ON_ERROR(sol_tcc->init_ok(), "SOLENOID", "TCC init not OK");
    ESP_RETURN_ON_ERROR(sol_mpc->init_ok(), "SOLENOID", "MPC init not OK");
    ESP_RETURN_ON_ERROR(sol_spc->init_ok(), "SOLENOID", "SPC init not OK");
    ESP_RETURN_ON_ERROR(sol_y3->init_ok(), "SOLENOID", "Y3 init not OK");
    ESP_RETURN_ON_ERROR(sol_y4->init_ok(), "SOLENOID", "Y4 init not OK");
    ESP_RETURN_ON_ERROR(sol_y5->init_ok(), "SOLENOID", "Y5 init not OK");
    xTaskCreate(update_solenoids, "LEDC-Update", 8192, nullptr, 10, nullptr);
    xTaskCreate(read_solenoids_i2s, "I2S-Reader", 8192*2, nullptr, 3, nullptr);
    //xTaskCreate(Solenoids::boot_solenoid_test, "Solenoid-Test", 8192, nullptr, 3, nullptr);
    return ESP_OK;
}