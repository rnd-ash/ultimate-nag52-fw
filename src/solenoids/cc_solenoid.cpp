#include "cc_solenoid.h"
#include "esp_log.h"
#include "nvs/module_settings.h"

ConstantCurrentSolenoid::ConstantCurrentSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms)
: PwmSolenoid(name, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->current_target = 0;
    this->counter = 0;
}

void ConstantCurrentSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    // Adjust PWM based on current feedback
    // Only do this every 10 steps (Every 10ms)
    uint16_t calc_pwm = this->pwm;
    if (this->current_target == 0) {
        calc_pwm = 0;
        this->counter = 0;
    } else {
        // Adapt first from previous current sample
        uint16_t read = this->get_current();
        if (this->prev_step_current_target >= 250) { // 250mA cut off
            uint16_t max = this->prev_step_max;
            float err = ((float)this->prev_step_current_target-(float)read)/(float)max;
            this->internal_trim_factor += (err/10.0);
            if (this->internal_trim_factor > 0.99) {
                this->internal_trim_factor = 0.99;
            } else if (this->internal_trim_factor < -0.99) {
                this->internal_trim_factor = -0.99;
            }
        }


        // Calculated wire resistance (Static 1 Ohm for PCB)
        //float resistance = 1 + (SOL_CURRENT_SETTINGS.cc_reference_resistance + (SOL_CURRENT_SETTINGS.cc_reference_resistance*temperature_factor));
        // vref_compensation is battery voltage compensation
        // Calculate maximum current through the wires given the current voltage and resistance of the wires
        this->prev_step_current_target = this->current_target;
        float max_current = ((float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)SOL_CURRENT_SETTINGS.cc_reference_resistance) / vref_compensation;
        this->prev_step_max = max_current;
        calc_pwm = MAX(0, (4096.0 * (this->prev_step_current_target/max_current)) * (1.0+this->internal_trim_factor));
        if (calc_pwm > 4096) {
            calc_pwm = 4096;
        }
    }
    if (calc_pwm != this->pwm) {
        ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, calc_pwm);
        ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
        this->pwm = calc_pwm;
    }
    this->counter++;
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