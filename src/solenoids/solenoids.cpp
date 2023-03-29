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

adc_cali_handle_t adc1_cal = nullptr;
uint16_t voltage = 12000;

Solenoid::Solenoid(const char *name, gpio_num_t pwm_pin, uint32_t frequency, ledc_channel_t channel, ledc_timer_t timer, adc_channel_t read_channel)
{
    this->channel = channel;
    this->timer = timer;
    this->name = name;
    this->adc_reading_current = 0;
    this->default_freq = frequency;
    this->adc_channel = read_channel;
    this->adc_sample_idx = 0;
    this->adc_total = 0;
    memset(this->adc_avgs, 0, sizeof(this->adc_avgs));
    esp_err_t ret = ESP_OK;
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE, // Low speed timer mode
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = timer,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK
    };

    const ledc_channel_config_t channel_cfg = {
        .gpio_num = pwm_pin,
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE, // Disable fade interrupt
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };


    //ESP_GOTO_ON_ERROR(adc1_config_channel_atten(this->adc_channel, adc_atten_t::ADC_ATTEN_DB_11), set_err, "SOLENOID", "Solenoid %s ADC1 init failed", name);
    // Set the timer configuration
    ESP_GOTO_ON_ERROR(ledc_timer_config(&timer_cfg), set_err, "SOLENOID", "Solenoid %s timer init failed", name);
    // Set PWM channel configuration
    ESP_GOTO_ON_ERROR(ledc_channel_config(&channel_cfg), set_err, "SOLENOID", "Failed to set LEDC channel for solenoid %s", name);
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SOLENOID", "Solenoid %s init OK!", name);
set_err:
    this->ready = ret;
}

void Solenoid::write_pwm_12_bit(uint16_t pwm_raw, bool voltage_compensate) {
    if (pwm_raw > 4096) {
        pwm_raw = 4096;
    }
    this->pwm_raw = pwm_raw;
    this->voltage_compensate = voltage_compensate;
}

uint16_t Solenoid::get_current() const {
    int v = 0;
    adc_cali_raw_to_voltage(adc1_cal, this->adc_reading_current, &v);
    //if (v <= adc1_cal.coeff_b) {
    //    v = 0; // Too small
    //}
    return v*pcb_gpio_matrix->sensor_data.current_sense_multi;
}

uint16_t Solenoid::get_pwm_raw()
{
    return this->pwm_raw;
}

uint16_t Solenoid::get_pwm_compensated() const
{   
    return this->pwm;
}

uint16_t Solenoid::diag_get_adc_peak_raw() {
    return this->adc_reading_current;
}


uint16_t Solenoid::get_current_avg() const
{   
    int v = 0;
    adc_cali_raw_to_voltage(adc1_cal, (float)this->adc_total/(float)SOLENOID_CURRENT_AVG_SAMPLES, &v);
    //if (v <= adc1_cal->) {
    //    v = 0; // Too small
    //}
    return v*pcb_gpio_matrix->sensor_data.current_sense_multi;
}

void Solenoid::__write_pwm() {
    if (this->pwm_raw == 0) {
        if (this->pwm_raw == this->pwm) { return; }
        ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, 0);
        ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
        this->pwm = 0;
    } else {
        if (this->voltage_compensate) {
            Sensors::read_vbatt(&voltage);
            this->pwm = (float)this->pwm_raw * SOLENOID_VREF / (float)voltage;
            if (this->pwm > 4096) {
                this->pwm = 4096; // Clamp to max
            }
        } else {
            this->pwm = this->pwm_raw;
        }
        ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, this->pwm);
        ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
    }
}

void Solenoid::__set_adc_reading(uint16_t c)
{
    this->adc_reading_current = c;
    this->adc_total -= this->adc_avgs[this->adc_sample_idx];
    this->adc_avgs[this->adc_sample_idx] = c;
    this->adc_total += c;
    this->adc_sample_idx = (this->adc_sample_idx+1)%SOLENOID_CURRENT_AVG_SAMPLES;
}

adc_channel_t Solenoid::get_adc_channel() const {
    return this->adc_channel;
}

esp_err_t Solenoid::init_ok() const
{
    return this->ready;
}

Solenoid *sol_y3 = nullptr;
Solenoid *sol_y4 = nullptr;
Solenoid *sol_y5 = nullptr;

Solenoid *sol_mpc = nullptr;
Solenoid *sol_spc = nullptr;
Solenoid *sol_tcc = nullptr;


uint8_t adc_read_buf[I2S_DMA_BUF_LEN];
bool first_read_complete = false;
bool monitor_all = false;

void read_solenoids_i2s(void*) {
    Solenoid* sol_batch[6] = { sol_mpc, sol_spc, sol_tcc, sol_y3, sol_y4, sol_y5};
    adc_continuous_handle_t c_handle = nullptr;
    adc_continuous_handle_cfg_t c_cfg = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 1024/SOC_ADC_DIGI_DATA_BYTES_PER_CONV
    };
    adc_continuous_new_handle(&c_cfg, &c_handle);
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 6,
        .sample_freq_hz = 600000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    for (int i = 0; i < 6; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = sol_batch[i]->get_adc_channel() & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    adc_continuous_config(c_handle, &dig_cfg);
    esp_err_t ret;
    memset(adc_read_buf, 0xcc, I2S_DMA_BUF_LEN);
    uint32_t out_len = 0;
    adc_continuous_start(c_handle);

    uint32_t samples[adc_channel_t::ADC_CHANNEL_9]; // Indexes all ADC channels like this
    uint64_t totals[adc_channel_t::ADC_CHANNEL_9];
    uint8_t channel;
    uint16_t value;
    while(true) {
        uint32_t read = 0;
        memset(samples, 0, sizeof(samples));
        memset(totals, 0, sizeof(totals));
        ret = adc_continuous_read(c_handle, adc_read_buf, I2S_DMA_BUF_LEN, &out_len, 1);
        if (ret == ESP_OK) {
            for (int i = 0; i < out_len; i+= SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_read_buf[i];
                uint32_t channel_number = p->type1.channel;
                uint32_t value = p->type1.data;
                if (value != 0) {
                    totals[channel_number]+=value;
                    samples[channel_number]++;
                }
            }
            for (int solenoid = 0; solenoid < 6; solenoid++) {
                uint8_t idx = (uint8_t)sol_batch[solenoid]->get_adc_channel(); // Channel index
                float avg = (samples[idx] == 0) ? 0 :  totals[idx] / (float)samples[idx];
                sol_batch[solenoid]->__set_adc_reading(avg);
            }
            //ESP_LOGI("ADC", "Read ok %lu bytes", out_len);
        }
        //vTaskDelay(1);
    }
}

/*
void read_solenoids_i2s(void*) {
    esp_log_level_set("I2S", esp_log_level_t::ESP_LOG_WARN); // Discard noisy I2S logs!
    Solenoid* sol_batch[6] = { sol_mpc, sol_spc, sol_tcc, sol_y3, sol_y4, sol_y5};
    i2s_config_t i2s_conf = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = 600000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_conf, 0, nullptr));
    ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_1, adc1_channel_t::ADC1_CHANNEL_0)); // Don't care about this
    vTaskDelay(10);
    // Enable ADC I2S. Then manipulate registers of SAR
    i2s_adc_enable(I2S_NUM_0);
    // Now modify SAR1 register so that it scans for ALL 6 channels at once
    // This lets us scan all 6 Solenoid feedback channels at one time!
    SYSCON.saradc_sar1_patt_tab[0] = 
        (((sol_batch[0]->get_adc_channel() << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << 24) |
        (((sol_batch[1]->get_adc_channel() << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << 16) |
        (((sol_batch[2]->get_adc_channel() << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << 8)  |
        (((sol_batch[3]->get_adc_channel() << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << 0);
    SYSCON.saradc_sar1_patt_tab[1] = 
        (((sol_batch[4]->get_adc_channel() << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << 24) |
        (((sol_batch[5]->get_adc_channel() << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << 16);
    SYSCON.saradc_ctrl.sar1_patt_len = 0x05; // 0x05 = 6 channel patterns stored in SAR registers
    SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV); // Invert SAR data to correct endienness
    ESP_LOGI("CC", "Starting");
    uint32_t samples[ADC_CHANNEL_MAX];
    uint64_t totals[ADC_CHANNEL_MAX];
    uint8_t channel;
    uint16_t value;
    while(true) {
        // Invert back data from ADC
        uint32_t read = 0;
        memset(samples, 0, sizeof(samples));
        memset(totals, 0, sizeof(totals));
        //for (int sample_id = 0; sample_id < 5; sample_id++) {
        i2s_read(I2S_NUM_0, glob_dma_buf, I2S_DMA_BUF_LEN*2, &read, portMAX_DELAY);
        for (int i = 0; i < read/2; i++) {
            channel = (glob_dma_buf[i] >> 12) & 0x07;
            value = glob_dma_buf[i] & 0xFFF;
            if (value != 0) {
                totals[channel]+=value;
                samples[channel]++;
            }
        }
        //}
        for (int solenoid = 0; solenoid < 6; solenoid++) {
            uint8_t idx = (uint8_t)sol_batch[solenoid]->get_adc_channel(); // Channel index
            float avg = (samples[idx] == 0) ? 0 :  totals[idx] / (float)samples[idx];
            sol_batch[solenoid]->__set_adc_reading(avg);
        }
        first_read_complete = true;
    }
}
*/

uint16_t Solenoids::get_solenoid_voltage() {
    return voltage;
}

void Solenoids::toggle_all_solenoid_current_monitoring(bool enable) {
    monitor_all = enable;
}

bool Solenoids::is_monitoring_all_solenoids() {
    return monitor_all;
}

void update_solenoids(void*) {
    Solenoid* sol_order[6] = { sol_mpc, sol_spc, sol_tcc, sol_y3, sol_y4, sol_y5 };
    while(true) {
        for (int i = 0; i < 6; i++) {
            sol_order[i]->__write_pwm();
        }
        vTaskDelay(1);
    }
}

float resistance_mpc = 5.0;
float resistance_spc = 5.0;
bool temp_cal = false;
int16_t temp_at_test = 25;

bool routine = false;
bool startup_ok = false;

bool Solenoids::init_routine_completed(void) {
    return routine;
}

// bool Solenoids::startup_test_ok() {
//     return startup_ok;
// }

void Solenoids::boot_solenoid_test(void*) {
    vTaskDelay(50);
    if(sol_spc->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "SPC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_mpc->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "MPC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_tcc->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "TCC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_y3->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y3 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_y4->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y4 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if(sol_y5->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y5 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }
    toggle_all_solenoid_current_monitoring(false);
    // Get resistance values from SPC and MPC
    if (get_solenoid_voltage() > 11000) {
        uint32_t c_total = 0;
        uint32_t b_total = 0;
        while(!first_read_complete){vTaskDelay(1);}
        sol_mpc->write_pwm_12_bit(4096, false);
        vTaskDelay(50);
        for (int i = 0; i < 10; i++) {
            b_total += get_solenoid_voltage();
            c_total += sol_mpc->get_current_avg();
            vTaskDelay(10);
        }
        ESP_LOGI("SOLENOID", "MPC Current: %lu mA at %lu mV", c_total/10, b_total/10);
        resistance_mpc = (float)b_total / (float)c_total;
        sol_mpc->write_pwm_12_bit(0);
        sol_spc->write_pwm_12_bit(4096, false);
        vTaskDelay(50);
        c_total = 0;
        b_total = 0;
        for (int i = 0; i < 10; i++) {
            b_total += get_solenoid_voltage();
            c_total += sol_spc->get_current_avg();
            vTaskDelay(10);
        }
        ESP_LOGI("SOLENOID", "SPC Current: %lu mA at %lu mV", c_total/10, b_total/10);
        resistance_spc = (float)b_total / (float)c_total;
        sol_mpc->write_pwm_12_bit(0);
        sol_spc->write_pwm_12_bit(0);
        ESP_LOGI("SOLENOID", "Resistance values. SPC: %.2f Ohms, MPC: %.2f Ohms", resistance_spc, resistance_mpc);
    }
    startup_ok = true;
    routine = true;
    vTaskDelete(NULL);
}

esp_err_t Solenoids::init_all_solenoids()
{
    adc_cali_line_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .atten = adc_atten_t::ADC_ATTEN_DB_11,
        .bitwidth = adc_bitwidth_t::ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_line_fitting(&cali, &adc1_cal);
    //esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_1, adc_atten_t::ADC_ATTEN_DB_11, adc_bits_width_t::ADC_WIDTH_BIT_12, 0, &adc1_cal);   
    // Read calibration for ADC1
    sol_y3 = new Solenoid("Y3", pcb_gpio_matrix->y3_pwm, 1000, ledc_channel_t::LEDC_CHANNEL_0, ledc_timer_t::LEDC_TIMER_0, ADC_CHANNEL_0);
    sol_y4 = new Solenoid("Y4", pcb_gpio_matrix->y4_pwm, 1000, ledc_channel_t::LEDC_CHANNEL_1, ledc_timer_t::LEDC_TIMER_0, ADC_CHANNEL_3);
    sol_y5 = new Solenoid("Y5", pcb_gpio_matrix->y5_pwm, 1000, ledc_channel_t::LEDC_CHANNEL_2, ledc_timer_t::LEDC_TIMER_0, ADC_CHANNEL_7);
    sol_mpc = new Solenoid("MPC", pcb_gpio_matrix->mpc_pwm, 1000, ledc_channel_t::LEDC_CHANNEL_3, ledc_timer_t::LEDC_TIMER_0, ADC_CHANNEL_6);
    sol_spc = new Solenoid("SPC", pcb_gpio_matrix->spc_pwm, 1000, ledc_channel_t::LEDC_CHANNEL_4, ledc_timer_t::LEDC_TIMER_0, ADC_CHANNEL_4);
    sol_tcc = new Solenoid("TCC", pcb_gpio_matrix->tcc_pwm, 100, ledc_channel_t::LEDC_CHANNEL_5, ledc_timer_t::LEDC_TIMER_1, ADC_CHANNEL_5);
    ESP_RETURN_ON_ERROR(ledc_fade_func_install(0), "SOLENOID", "Could not insert LEDC_FADE");
    ESP_RETURN_ON_ERROR(sol_tcc->init_ok(), "SOLENOID", "TCC init not OK");
    ESP_RETURN_ON_ERROR(sol_mpc->init_ok(), "SOLENOID", "MPC init not OK");
    ESP_RETURN_ON_ERROR(sol_spc->init_ok(), "SOLENOID", "SPC init not OK");
    ESP_RETURN_ON_ERROR(sol_y3->init_ok(), "SOLENOID", "Y3 init not OK");
    ESP_RETURN_ON_ERROR(sol_y4->init_ok(), "SOLENOID", "Y4 init not OK");
    ESP_RETURN_ON_ERROR(sol_y5->init_ok(), "SOLENOID", "Y5 init not OK");
    toggle_all_solenoid_current_monitoring(true);
    Sensors::read_vbatt(&voltage);
    xTaskCreate(update_solenoids, "LEDC-Update", 4096, nullptr, 10, nullptr);
    xTaskCreate(read_solenoids_i2s, "I2S-Reader", 16000, nullptr, 3, nullptr);
    xTaskCreate(Solenoids::boot_solenoid_test, "Solenoid-Test", 8192, nullptr, 3, nullptr);
    return ESP_OK;
}