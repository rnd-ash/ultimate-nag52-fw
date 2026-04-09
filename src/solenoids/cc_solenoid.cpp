#include "cc_solenoid.h"
#include "esp_log.h"
#include "nvs/module_settings.h"
#include <string.h>
#include "sensors.h"


ConstantCurrentSolenoid::ConstantCurrentSolenoid(const char* name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms)
    : PwmSolenoid(name, ledc_timer, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->saved_current_target = 0;
    this->current_target = 0;
    this->channel = channel;
    this->pid[0] = 0;
    this->pid[1] = 0;
    this->pid[2] = 0;
}

void ConstantCurrentSolenoid::__write_pwm(float vref_compensation, float temperature_factor, bool stop_compensation) {
    // Moved to ConstantCurrentSolenoid::update_when_reading
}

void ConstantCurrentSolenoid::set_current_target(uint16_t target_ma) {
    this->current_target = target_ma;
}

uint16_t ConstantCurrentSolenoid::get_current_target() {
    return this->current_target;
}

float ConstantCurrentSolenoid::get_trim() {
    return 1.0 + ((float)this->trim_pwm / 4096.0);
}

uint16_t ConstantCurrentSolenoid::get_current(void) const {
    uint16_t raw = PwmSolenoid::get_current();
    return MAX(0, raw + SOL_CURRENT_SETTINGS.cc_offset_ma);
}

void ConstantCurrentSolenoid::update_when_reading(uint16_t battery) {
    // Pull in new values
    uint16_t prev_current_req = this->saved_current_target;
    if (battery > 9000 && this->current_target != 0) {
        // Assume 14.4V for stability
        int32_t max_current_ma = SOL_CURRENT_SETTINGS.cc_vref_solenoid / SOL_CURRENT_SETTINGS.cc_reference_resistance;
        uint16_t observed_current = this->get_current();
        this->saved_current_target = this->current_target;
        int32_t error = 1000 * ((int32_t)prev_current_req - (int32_t)observed_current) / (float)max_current_ma;
        if (prev_current_req >= 200 && observed_current >= 200) {
            // PID Compensate
            int32_t p = SOL_CURRENT_SETTINGS.cc_pid_p;
            int32_t i = SOL_CURRENT_SETTINGS.cc_pid_i;
            int32_t d = SOL_CURRENT_SETTINGS.cc_pid_d;

            int32_t p_v = (p * error) / 1000;
            pid[1] += error;
            pid[1] = MAX(INT16_MIN, MIN(INT16_MAX, pid[1]));

            int32_t i_v = (i * pid[1]) / 1000;
            int32_t d_v = (d * (error - pid[0])) / 1000;
            pid[0] = error;
            trim_pwm = MAX(INT16_MIN, MIN(INT16_MAX, p_v + i_v + d_v));
        }
        float mult = ((float)this->current_target / (float)max_current_ma);
        int16_t targ_pwm = MAX(0, (4096.0 * mult));
        targ_pwm += ((float)this->trim_pwm * (mult));
        this->pwm = MAX(0, MIN(targ_pwm, 4096));
    }
    else {
        this->saved_current_target = 0;
        this->pwm = 0;
        // Don't reset PID information since this slows down reaction when solenoid comes back online
        //this->trim_pwm = 0;
        //this->pid[0] = 0;
        //this->pid[1] = 0;
        //this->pid[2] = 0;
    }
    ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, this->pwm);
    ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
}