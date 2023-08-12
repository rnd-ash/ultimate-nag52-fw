#include "cc_solenoid.h"
#include "nvs/module_settings.h"
#include "esp_log.h"

ConstantCurrentSolenoid::ConstantCurrentSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint8_t current_samples)
: PwmSolenoid(name, pwm_pin, channel, read_channel, current_samples) {
    this->current_target = 0;
}

uint8_t c = 0;

void ConstantCurrentSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    // Adjust PWM based on current feedback
    // Only do this every 10 steps (Every 10ms)
    uint16_t calc_pwm = this->pwm;
    if (this->current_target == 0) {
        calc_pwm = 0;
        c = 0;
    } else {
        this->current_avg_samples->add_sample((int32_t)this->get_current());
        // Calculated wire resistance (Static 1 Ohm for PCB)
        float resistance = 1 + (SOL_CURRENT_SETTINGS.cc_reference_resistance + (SOL_CURRENT_SETTINGS.cc_reference_resistance*temperature_factor));
        // vref_compensation is battery voltage compensation
        // Calculate maximum current through the wires given the current voltage and resistance of the wires
        float max_current = ((float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / resistance) / vref_compensation;

        uint16_t req_pwm = 4096.0 * (this->current_target/max_current) * this->internal_trim_factor;
        calc_pwm = req_pwm;

        // Learning correction on sampled current
        if (c >= 10 && this->current_avg_samples->has_full_samples()) {
            uint16_t read = this->current_avg_samples->get_average();
            float target_err_multi = ((float)this->current_target/(float)read);
            // 1.0 means perfect
            // < 1.0 means too high
            // > 1.0 means too low
            //if (target_err_multi < 0.98 || target_err_multi > 1.02) {
                float err_2 = target_err_multi-this->internal_trim_factor;
                if (err_2 > 0.005) {
                    err_2 = 0.005;
                } else if (err_2 <= -0.005) {
                    err_2 = -0.005;
                }
                this->internal_trim_factor += err_2;
            //}
            c = 0;
        }
    }
    if (calc_pwm != this->pwm) {
        ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, calc_pwm);
        ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
        this->pwm = calc_pwm;
    }
    c++;
}

void ConstantCurrentSolenoid::set_current_target(uint16_t target_ma) {
    if (target_ma != this->current_target) {
        c = 0;
    }
    this->current_target = 250;
}

uint16_t ConstantCurrentSolenoid::get_current_target() {
    return this->current_target;
}

float ConstantCurrentSolenoid::get_trim() {
    return this->internal_trim_factor;
}