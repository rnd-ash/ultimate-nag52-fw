#include "pwm_solenoid.h"

#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"
#include "board_config.h"
#include "string.h"
#include "../nvs/module_settings.h"
#include "esp_check.h"

uint16_t voltage = 12000;
adc_cali_handle_t adc1_cal = nullptr;

PwmSolenoid::PwmSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms)
{
    this->channel = channel;
    this->name = name;
    this->adc_channel = read_channel;
    this->pwm_phase_period_ms = phase_duration_ms;
    esp_err_t ret = ESP_OK;


    const ledc_channel_config_t channel_cfg = {
        .gpio_num = pwm_pin,
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE, // Disable fade interrupt
        .timer_sel = ledc_timer,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0u
        }
    };

    ledc_timer_config_t SOLENOID_TIMER_CFG = {
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE, // Low speed timer mode
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = ledc_timer,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    // Set the timer configuration
    ESP_GOTO_ON_ERROR(ledc_timer_config(&SOLENOID_TIMER_CFG), set_err, "SOLENOID", "Solenoid %s timer init failed", name);
    // Set PWM channel configuration
    ESP_GOTO_ON_ERROR(ledc_channel_config(&channel_cfg), set_err, "SOLENOID", "Failed to set LEDC channel for solenoid %s", name);
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SOLENOID", "Solenoid %s init OK!", name);
set_err:
    this->ready = ret;
}

uint16_t PwmSolenoid::get_current() const {
    uint32_t raw = this->current_reading;
    uint16_t ret = 0;
    if (0 != raw) {
        adc_cali_raw_to_voltage(adc1_cal, raw, (int*)&ret);
    }
    return ret * pcb_gpio_matrix->sensor_data.current_sense_multi;
}

uint16_t PwmSolenoid::get_pwm_raw()
{
    return this->pwm_raw;
}

SolenoidTestReading PwmSolenoid::get_full_on_current_reading() {
#define NUM_SAMPLES 50
    uint32_t v_total = 0;
    uint32_t c_total = 0;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, this->channel, 4096);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, this->channel);
    vTaskDelay(200);
    for(uint8_t i = 0; i < NUM_SAMPLES; i++) {
        c_total += this->get_current();
        v_total += voltage;
        vTaskDelay(5);
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, this->channel, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, this->channel);
    vTaskDelay(100);
    return SolenoidTestReading {
        .avg_voltage = (uint16_t)(v_total / NUM_SAMPLES),
        .avg_current = (uint16_t)(c_total / NUM_SAMPLES)
    };
}

uint16_t PwmSolenoid::get_pwm_compensated() const
{   
    return this->pwm;
}

void PwmSolenoid::__set_adc_reading(uint16_t c)
{
    this->current_reading = c;
}

adc_channel_t PwmSolenoid::get_adc_channel() const {
    return this->adc_channel;
}

esp_err_t PwmSolenoid::init_ok() const
{
    return this->ready;
}

uint16_t PwmSolenoid::get_ledc_pwm() {
    return ledc_get_duty(LEDC_HIGH_SPEED_MODE, this->channel);
}

uint16_t PwmSolenoid::get_pwm_phase_time() const {
    return this->pwm_phase_period_ms;
}


esp_err_t SolenoidSetup::init_adc() {
    const adc_cali_line_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .atten = adc_atten_t::ADC_ATTEN_DB_11,
        .bitwidth = adc_bitwidth_t::ADC_BITWIDTH_12,
        .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_line_fitting(&cali, &adc1_cal), "SOLENOID", "Failed to create ADC cal");
    return ESP_OK;
}