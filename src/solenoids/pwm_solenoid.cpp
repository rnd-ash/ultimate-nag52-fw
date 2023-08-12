#include "pwm_solenoid.h"

#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"
#include "board_config.h"
#include "string.h"
#include "../nvs/module_settings.h"
#include "esp_check.h"

uint16_t voltage = 12000;
uint16_t min_adc_v_reading = 0;

const ledc_timer_t SOLENOID_TIMER = ledc_timer_t::LEDC_TIMER_0;

const ledc_timer_config_t SOLENOID_TIMER_CFG = {
    .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE, // Low speed timer mode
    .duty_resolution = LEDC_TIMER_12_BIT,
    .timer_num = SOLENOID_TIMER,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
};

PwmSolenoid::PwmSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint8_t current_samples)
{
    this->channel = channel;
    this->name = name;
    this->adc_channel = read_channel;
    esp_err_t ret = ESP_OK;


    const ledc_channel_config_t channel_cfg = {
        .gpio_num = pwm_pin,
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE, // Disable fade interrupt
        .timer_sel = SOLENOID_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    // Set the timer configuration
    ESP_GOTO_ON_ERROR(ledc_timer_config(&SOLENOID_TIMER_CFG), set_err, "SOLENOID", "Solenoid %s timer init failed", name);
    // Set PWM channel configuration
    ESP_GOTO_ON_ERROR(ledc_channel_config(&channel_cfg), set_err, "SOLENOID", "Failed to set LEDC channel for solenoid %s", name);

    this->current_avg_samples = new MovingAverage(current_samples);
    if (!this->current_avg_samples->init_ok()) {
        ESP_LOGW("SOLENOID", "Sol %s moving average buffer failed to allocate", name);
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SOLENOID", "Solenoid %s init OK!", name);
set_err:
    this->ready = ret;
}

uint16_t PwmSolenoid::get_current() const {
    uint16_t ret = this->adc_now_read;
    if (nullptr != this->current_avg_samples) {
        ret = this->current_avg_samples->get_average();
    }
    return ret * pcb_gpio_matrix->sensor_data.current_sense_multi;
}

uint16_t PwmSolenoid::get_pwm_raw()
{
    return this->pwm_raw;
}

uint16_t PwmSolenoid::get_pwm_compensated() const
{   
    return this->pwm;
}

void PwmSolenoid::__set_adc_reading(uint16_t c)
{
    this->adc_now_read = c;
    if (nullptr != this->current_avg_samples) {
        this->current_avg_samples->add_sample(c);
    }
}

adc_channel_t PwmSolenoid::get_adc_channel() const {
    return this->adc_channel;
}

esp_err_t PwmSolenoid::init_ok() const
{
    return this->ready;
}