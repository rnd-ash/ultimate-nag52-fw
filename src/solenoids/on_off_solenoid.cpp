#include "on_off_solenoid.h"
#include "esp_check.h"
#include "tcu_maths.h"
#include "clock.hpp"

OnOffSolenoid::OnOffSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint32_t on_time_ms, uint16_t hold_pwm, uint16_t phase_duration_ms)
: PwmSolenoid(name, ledc_timer, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->state = false;
    this->target_hold_pwm = hold_pwm;
    this->target_on_time = on_time_ms;
}

void OnOffSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    uint16_t calc_pwm = 0;
    if (this->state) {
        // Solenoid should be on!
        if (this->holding || GET_CLOCK_TIME() - this->on_time_ms >= this->target_on_time) { // this->holding call short circuits, so we check this first before checking against clock!
            this->holding = true;
            calc_pwm = this->target_hold_pwm * vref_compensation;
        } else {
            calc_pwm = 0xFFFF;
        }
    }
    ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, calc_pwm);
    ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
}

void OnOffSolenoid::on() {
    if (!this->state) {
        this->on_time_ms = GET_CLOCK_TIME();
        this->holding = false;
    }
    this->state = true;
}

void OnOffSolenoid::off() {
    this->on_time_ms = 0;
    this->state = false;
    this->holding = false;
}

bool OnOffSolenoid::is_on() {
    return this->state;
}

bool OnOffSolenoid::is_max_on() {
    return this->state && !this->holding;
}