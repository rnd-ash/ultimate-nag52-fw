#include "cc_solenoid.h"
#include "esp_log.h"
#include "nvs/module_settings.h"
#include <string.h>
#include "sensors.h"

float mpc_sol_trim_factor = 0.0;

ConstantCurrentSolenoid::ConstantCurrentSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms, bool is_mpc)
: PwmSolenoid(name, ledc_timer, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->saved_current_target = 0;
    this->current_target = 0;
    this->use_global_cc = !is_mpc;
    this->channel = channel;
}

void ConstantCurrentSolenoid::__write_pwm(float vref_compensation, float temperature_factor, bool stop_compensation) {
    // Moved to ConstantCurrentSolenoid::update_when_reading
}

void ConstantCurrentSolenoid::set_current_target(uint16_t target_ma) {
    if (target_ma > 1500) {
        target_ma = 1500;
    }
    this->current_target = target_ma;
    //this->c = 0;
}

uint16_t ConstantCurrentSolenoid::get_current_target() {
    return this->current_target;
}

float ConstantCurrentSolenoid::get_trim() {
    return 1.0+this->internal_trim_factor;
}

void ConstantCurrentSolenoid::update_when_reading(uint16_t battery) {
    if(battery > 9000) {
        // When the previous cycle was ran
        float max_current = ((float)battery / (float)SOL_CURRENT_SETTINGS.cc_reference_resistance);
        uint16_t current_targ_when_reading = this->saved_current_target;
        this->saved_current_target = this->current_target;
        if (correct_cycle) {
            int16_t error = MIN(current_targ_when_reading - this->get_current(), 200);
            uint16_t jump = abs(this->saved_current_target - current_targ_when_reading);
            if (current_targ_when_reading >= 200 && this->current_target >= 200 && abs(error) > 10 && jump <= 500) {
                // Compensate
                
                // 1. Error as a proportion of max current
                float error_f = (float)error / max_current;
                // 2. Set trim
                this->internal_trim_factor += error_f/2;
            }
        }
        if (this->internal_trim_factor > 0.5) {
            this->internal_trim_factor = 0.5;
        } else if (this->internal_trim_factor < -0.5) {
            this->internal_trim_factor = -0.5;
        }
        if (!use_global_cc) {
            mpc_sol_trim_factor = this->internal_trim_factor;
        } else {
            if (this->current_target == 0) {
                this->internal_trim_factor = mpc_sol_trim_factor;
            }
        }
        uint16_t targ_pwm = 0;
        if (this->saved_current_target != 0 && this->current_target != 0) {
            float step_per_pwm = 4096.0 / max_current;
            float calc = step_per_pwm * this->saved_current_target;
            targ_pwm = calc * this->internal_trim_factor;
            targ_pwm = MAX(0, (4096.0 * (this->current_target/max_current)));
            // RMS trim factor
            targ_pwm += targ_pwm * (this->internal_trim_factor * (max_current/this->current_target));
        }
        this->pwm = MIN(targ_pwm, 4096);
        ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, targ_pwm);
        ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
        this->correct_cycle = !this->correct_cycle;
    }
}