#include "pwm_solenoid.h"

#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"
#include "board_config.h"
#include "string.h"
#include "../nvs/module_settings.h"
#include "esp_check.h"

uint16_t voltage = 12000;
uint16_t min_adc_v_reading = 0;
uint16_t min_adc_raw_reading = 0;

adc_cali_handle_t adc1_cal = nullptr;

const ledc_timer_t SOLENOID_TIMER = ledc_timer_t::LEDC_TIMER_0;

const ledc_timer_config_t SOLENOID_TIMER_CFG = {
    .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE, // Low speed timer mode
    .duty_resolution = LEDC_TIMER_12_BIT,
    .timer_num = SOLENOID_TIMER,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
};

PwmSolenoid::PwmSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint8_t current_samples, uint8_t avg_samples)
{
    this->channel = channel;
    this->name = name;
    this->adc_channel = read_channel;
    this->c_readings = new MovingUnsignedAverage(avg_samples);
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
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SOLENOID", "Solenoid %s init OK!", name);
set_err:
    this->ready = ret;
}

uint16_t PwmSolenoid::get_current() const {
    uint32_t ret = this->c_readings->get_average();
    if (ret > min_adc_raw_reading) {
        adc_cali_raw_to_voltage(adc1_cal, ret, (int*)&ret);
    } else {
        ret = 0;
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
    this->c_readings->add_sample(c);
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


esp_err_t SolenoidSetup::init_adc() {
    const adc_cali_line_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .atten = adc_atten_t::ADC_ATTEN_DB_11,
        .bitwidth = adc_bitwidth_t::ADC_BITWIDTH_12,
        .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_line_fitting(&cali, &adc1_cal), "SOLENOID", "Failed to create ADC cal");
    // Set the minimum ADC Reading in mV
    ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(adc1_cal, 0, (int*)&min_adc_v_reading), "SOLENOID", "Failed to get ADC min value");
    int test = 0;
    for (uint16_t i = 0; i < 1024; i++) {
        adc_cali_raw_to_voltage(adc1_cal, i, &test);
        if (test > min_adc_v_reading) {
            min_adc_raw_reading = test-1;
            break;
        }
    }
    ESP_LOGI("SOLENOIDS", "Raw min for ADC1 is %d - %d", min_adc_raw_reading, min_adc_v_reading);
    return ESP_OK;
}