#include "on_off_solenoid.h"
#include "esp_check.h"
#include "tcu_maths.h"

OnOffSolenoid::OnOffSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint32_t on_time_ms, uint16_t hold_pwm, uint8_t current_samples)
: PwmSolenoid(name, pwm_pin, channel, read_channel, current_samples) {
    this->state = false;
    this->target_hold_pwm = hold_pwm;
    this->target_on_time = on_time_ms;
}

void OnOffSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    uint16_t calc_pwm = 0;
    if (this->state) {
        // Solenoid should be on!
        if (this->holding || (esp_timer_get_time()/1000) - this->on_time_ms >= this->target_on_time) { // this->holding call short circuits, so we check this first before checking against clock!
            this->holding = true;
            calc_pwm = this->target_hold_pwm * vref_compensation;
        } else {
            calc_pwm = 0xFFFF;
        }
    }
    if (calc_pwm != this->pwm) {
        ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, calc_pwm);
        ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
        this->pwm = calc_pwm;
    }
}

void OnOffSolenoid::on() {
    if (!this->state) {
        this->on_time_ms = esp_timer_get_time()/1000;
        this->holding = false;
    }
    this->state = true;
}

void OnOffSolenoid::off() {
    this->state = false;
    this->holding = false;
}

bool OnOffSolenoid::is_on() {
    return this->state;
}