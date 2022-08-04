#include "solenoids.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "pins.h"
#include "../sensors.h"
#include "soc/syscon_periph.h"
#include "soc/i2s_periph.h"
#include "string.h"
#include "driver/adc_deprecated.h"

esp_adc_cal_characteristics_t adc1_cal;
uint16_t voltage = 12000;

Solenoid::Solenoid(const char *name, gpio_num_t pwm_pin, uint32_t frequency, ledc_channel_t channel, ledc_timer_t timer, adc1_channel_t read_channel)
{
    this->channel = channel;
    this->timer = timer;
    this->ready = true; // Assume ready unless error!
    this->name = name;
    this->adc_reading_current = 0;
    this->default_freq = frequency;
    this->adc_channel = read_channel;
    this->adc_sample_idx = 0;
    this->adc_total = 0;
    memset(this->adc_avgs, 0, sizeof(this->adc_avgs));

    adc1_config_channel_atten(this->adc_channel, adc_atten_t::ADC_ATTEN_DB_11);

    ledc_timer_config_t timer_cfg = {
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE, // Low speed timer mode
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = timer,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK
    };
    // Set the timer configuration
    esp_err_t res = ledc_timer_config(&timer_cfg);
    if (res != ESP_OK)
    {
        this->ready = false;
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Solenoid %s timer init failed. Status code %d!", name, res);
        return;
    }

    // Set PWM channel configuration
    ledc_channel_config_t channel_cfg = {
        .gpio_num = pwm_pin,
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE, // Disable fade interrupt
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };

    res = ledc_channel_config(&channel_cfg);
    if (res != ESP_OK)
    {
        this->ready = false;
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Solenoid %s channel init failed. Status code %d!", name, res);
        return;
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SOLENOID", "Solenoid %s init OK!", name);
}

void Solenoid::write_pwm_12_bit(uint16_t pwm_raw, bool voltage_compensate) {
    if (pwm_raw > 4096) {
        pwm_raw = 4096;
    }
    this->pwm_raw = pwm_raw;
    this->voltage_compensate = voltage_compensate;
}

uint16_t Solenoid::get_current() {
    uint32_t v = esp_adc_cal_raw_to_voltage(this->adc_reading_current, &adc1_cal);
    if (v <= adc1_cal.coeff_b) {
        v = 0; // Too small
    }
#ifdef BOARD_V2
        return v;
    #else
        return v*2;
    #endif
}

uint16_t Solenoid::get_pwm_raw()
{
    return this->pwm_raw;
}

uint16_t Solenoid::get_pwm_compensated()
{   
    return this->pwm;
}

uint16_t Solenoid::diag_get_adc_peak_raw() {
    return this->adc_reading_current;
}


uint16_t Solenoid::get_current_avg()
{   
    uint32_t v = esp_adc_cal_raw_to_voltage((float)this->adc_total/(float)SOLENOID_CURRENT_AVG_SAMPLES, &adc1_cal);
    if (v <= adc1_cal.coeff_b) {
        v = 0; // Too small
    }
#ifdef BOARD_V2
        return v;
#else
        return v*2;
#endif
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

adc1_channel_t Solenoid::get_adc_channel() {
    return this->adc_channel;
}

bool Solenoid::init_ok() const
{
    return this->ready;
}

Solenoid *sol_y3 = nullptr;
Solenoid *sol_y4 = nullptr;
Solenoid *sol_y5 = nullptr;

Solenoid *sol_mpc = nullptr;
Solenoid *sol_spc = nullptr;
Solenoid *sol_tcc = nullptr;


uint16_t* buf = nullptr;
uint16_t glob_dma_buf[I2S_DMA_BUF_LEN];
bool first_read_complete = false;
bool monitor_all = false;

void read_solenoids_i2s(void*) {
    esp_log_level_set("I2S", esp_log_level_t::ESP_LOG_WARN); // Discard noisy I2S logs!
    Solenoid* sol_batch[6] = { sol_mpc, sol_spc, sol_tcc, sol_y3, sol_y4, sol_y5};
    esp_err_t e;
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

bool Solenoids::init_all_solenoids()
{
    esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_1, adc_atten_t::ADC_ATTEN_DB_11, adc_bits_width_t::ADC_WIDTH_BIT_12, 0, &adc1_cal);   
    // Read calibration for ADC1
    sol_y3 = new Solenoid("Y3", PIN_Y3_PWM, 1000, ledc_channel_t::LEDC_CHANNEL_0, ledc_timer_t::LEDC_TIMER_0, ADC1_CHANNEL_0);
    sol_y4 = new Solenoid("Y4", PIN_Y4_PWM, 1000, ledc_channel_t::LEDC_CHANNEL_1, ledc_timer_t::LEDC_TIMER_0, ADC1_CHANNEL_3);
    sol_y5 = new Solenoid("Y5", PIN_Y5_PWM, 1000, ledc_channel_t::LEDC_CHANNEL_2, ledc_timer_t::LEDC_TIMER_0, ADC1_CHANNEL_7);
    sol_mpc = new Solenoid("MPC", PIN_MPC_PWM, 1000, ledc_channel_t::LEDC_CHANNEL_3, ledc_timer_t::LEDC_TIMER_0, ADC1_CHANNEL_6);
    sol_spc = new Solenoid("SPC", PIN_SPC_PWM, 1000, ledc_channel_t::LEDC_CHANNEL_4, ledc_timer_t::LEDC_TIMER_0, ADC1_CHANNEL_4);
    sol_tcc = new Solenoid("TCC", PIN_TCC_PWM, 100, ledc_channel_t::LEDC_CHANNEL_5, ledc_timer_t::LEDC_TIMER_1, ADC1_CHANNEL_5);
    esp_err_t res = ledc_fade_func_install(0);
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Could not insert LEDC_FADE: %s", esp_err_to_name(res));
        return false;
    }
    if (!(
        sol_tcc->init_ok() && 
        sol_mpc->init_ok() && 
        sol_spc->init_ok() && 
        sol_y3->init_ok() && 
        sol_y4->init_ok() && 
        sol_y5->init_ok())
    ) { // Init error, don't do anything else
        return false;
    }
    toggle_all_solenoid_current_monitoring(true);
    vTaskDelay(50);
    xTaskCreate(update_solenoids, "LEDC-Update", 4096, nullptr, 10, nullptr);
    xTaskCreate(read_solenoids_i2s, "I2S-Reader", 16000, nullptr, 3, nullptr);
    if(sol_spc->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "SPC drawing too much current when off!");
        return false;
    }

    if(sol_mpc->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "MPC drawing too much current when off!");
        return false;
    }

    if(sol_tcc->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "TCC drawing too much current when off!");
        return false;
    }

    if(sol_y3->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y3 drawing too much current when off!");
        return false;
    }

    if(sol_y4->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y4 drawing too much current when off!");
        return false;
    }

    if(sol_y5->get_current_avg() > 500) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y5 drawing too much current when off!");
        return false;
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
        ESP_LOGI("SOLENOID", "MPC Current: %d mA at %d mV", b_total/10, c_total/10);
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
        ESP_LOGI("SOLENOID", "SPC Current: %d mA at %d mV", b_total/10, c_total/10);
        resistance_spc = (float)b_total / (float)c_total;
        sol_mpc->write_pwm_12_bit(0);
        sol_spc->write_pwm_12_bit(0);
        ESP_LOGI("SOLENOID", "Resistance values. SPC: %.2f Ohms, MPC: %.2f Ohms", resistance_spc, resistance_mpc);
    }
    return true;
}