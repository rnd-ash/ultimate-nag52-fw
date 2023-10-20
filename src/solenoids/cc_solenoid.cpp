#include "cc_solenoid.h"
#include "esp_log.h"
#include "nvs/module_settings.h"

ConstantCurrentSolenoid::ConstantCurrentSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms)
: PwmSolenoid(name, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->current_target = 0;
    memset(&this->old_current_targets, 0x00, sizeof(this->old_current_targets));
    this->current_target_idx = 0;
}

void ConstantCurrentSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    // Adjust PWM based on current feedback
    // Only do this every 4 steps (Every 4ms)
    
    // Add current targets
    int16_t current_delta = this->current_target - this->old_current_targets[current_target_idx];
    this->old_current_targets[current_target_idx] = this->current_target;
    this->current_target_idx = (this->current_target_idx + 1) % 10;

    uint16_t calc_pwm = this->pwm;
    if (this->current_target == 0) {
        calc_pwm = 0;
    } else {
        // Adapt first from previous current sample
        float max_current = ((float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)SOL_CURRENT_SETTINGS.cc_reference_resistance) / vref_compensation;

        float delta_as_percent = (float)current_delta / (float)max_current;

        uint16_t read = this->get_current();

        // 250mA sense cuttoff (To avoid noise), and only when current step delta is low such that the solenoid
        // can respond within the time window
        if (this->current_target_at_report_time >= 250 && abs(current_delta) <= 100) { // 250mA cut off
            // Large enough error, adapt!
            if (abs(this->current_target_at_report_time - read) > 10) {
                float err = (((float)this->current_target_at_report_time-(float)read)/max_current);
                // Don't over correct for error when current delta is high, so we can scale the error correction
                this->internal_trim_factor += err/interpolate_int(abs(current_delta), 20, 1000, 0, 100);
                if (this->internal_trim_factor > 0.99) {
                    this->internal_trim_factor = 0.99;
                } else if (this->internal_trim_factor < -0.99) {
                    this->internal_trim_factor = -0.99;
                }
            }
            // Remove old current reading (ADC thread will refresh this value)
            this->current_target_at_report_time = 0;
        }
        
        calc_pwm = MAX(0, (4096.0 * (this->current_target/max_current)));
        // RMS trim factor
        calc_pwm += calc_pwm * (this->internal_trim_factor * (max_current/this->current_target));

        //if (delta_as_percent < -0.1) {
        //    // Fast decrease for a split second so that current drops sharply
        //    calc_pwm /= interpolate_float(delta_as_percent, 10, 1, -0.5, -0.1, InterpType::Linear);
        //} else if (delta_as_percent > 0.1) {
        //    // Fast increase for a split second so that current ramps up sharply
        //    calc_pwm *= interpolate_float(delta_as_percent, 1, 1.25, 0.1, 0.5, InterpType::Linear);
        //}

        if (calc_pwm > 4096) {
            calc_pwm = 4096;
        }
    }

    if (calc_pwm != this->pwm) {
        ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, calc_pwm);
        ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
        this->pwm = calc_pwm;
    }
}

void ConstantCurrentSolenoid::set_current_target(uint16_t target_ma) {
    if (target_ma > 1500) {
        target_ma = 1500;
    }
    this->current_target = target_ma;
}

uint16_t ConstantCurrentSolenoid::get_current_target() {
    return this->current_target;
}

float ConstantCurrentSolenoid::get_trim() {
    return 1.0+this->internal_trim_factor;
}

void ConstantCurrentSolenoid::set_target_current_when_reading() {
    this->current_target_at_report_time = this->current_target;
}