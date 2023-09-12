#include "cc_solenoid.h"
#include "esp_log.h"
#include "nvs/module_settings.h"

ConstantCurrentSolenoid::ConstantCurrentSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint8_t current_samples)
: PwmSolenoid(name, pwm_pin, channel, read_channel, current_samples, 10) {
    this->current_target = 0;
    this->counter = 0;
    this->max_current_samples = new MovingUnsignedAverage(current_samples);
    this->req_samples = new MovingUnsignedAverage(current_samples);
}

void ConstantCurrentSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    // Adjust PWM based on current feedback
    // Only do this every 10 steps (Every 10ms)
    uint16_t calc_pwm = this->pwm;
    if (this->current_target == 0) {
        calc_pwm = 0;
        this->counter = 0;
        this->max_current_samples->reset();
        this->req_samples->reset();
    } else {
        // Calculated wire resistance (Static 1 Ohm for PCB)
        //float resistance = 1 + (SOL_CURRENT_SETTINGS.cc_reference_resistance + (SOL_CURRENT_SETTINGS.cc_reference_resistance*temperature_factor));
        // vref_compensation is battery voltage compensation
        // Calculate maximum current through the wires given the current voltage and resistance of the wires
        float max_current = ((float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)SOL_CURRENT_SETTINGS.cc_reference_resistance) / vref_compensation;
        calc_pwm = (4096.0 * (this->current_target/max_current)) * (1.0+this->internal_trim_factor);
        this->max_current_samples->add_sample(max_current);
        this->req_samples->add_sample(this->current_target);
        // Learning correction on sampled current
        if (this->current_target > 300) {
            uint16_t read = this->get_current();
            if (read > 300) {
                uint16_t max = this->max_current_samples->get_average();
                uint16_t req = this->req_samples->get_average();
                float err = ((float)req-(float)read)/(float)max;
                this->internal_trim_factor += (err/10.0);
                this->counter = 0;
            }
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