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

// The shared solenoid LEDC timer is configured for LEDC_TIMER_12_BIT, so a duty
// of 4096 is 100%. Anything larger is out of range for the timer resolution -
// writing it relies on unspecified driver/hardware behaviour, and would be
// rejected outright if ledc_set_duty() ever gained range validation.
#define SOLENOID_LEDC_FULL_DUTY (4096u)

void OnOffSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    uint32_t calc_pwm = 0u;
    if (this->state) {
        // Solenoid should be on!
        if (this->holding || GET_CLOCK_TIME() - this->on_time_ms >= this->target_on_time) { // this->holding call short circuits, so we check this first before checking against clock!
            this->holding = true;
            // Computed in a wide type: a low battery voltage drives
            // vref_compensation above 1.0, and target_hold_pwm * that can exceed
            // the 12 bit range (it overflowed uint16_t entirely below ~3V).
            float scaled = (float)this->target_hold_pwm * vref_compensation;
            if (!(scaled > 0.0f)) { // Also catches NaN
                scaled = 0.0f;
            }
            calc_pwm = (scaled >= (float)SOLENOID_LEDC_FULL_DUTY)
                ? SOLENOID_LEDC_FULL_DUTY
                : (uint32_t)scaled;
        } else {
            // Inrush phase - hold the solenoid fully on.
            // This used to write 0xFFFF, which is 16x the timer's full scale.
            calc_pwm = SOLENOID_LEDC_FULL_DUTY;
        }
    }
    // Keep the reported duty in step with what is actually driven. Without this
    // the diagnostic readouts for Y3/Y4/Y5 (RLI solenoid status, EGS emulation)
    // always reported 0, because nothing ever wrote pwm/pwm_raw.
    this->pwm_raw = (uint16_t)calc_pwm;
    this->pwm = (uint16_t)calc_pwm;
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

/* unused */
// bool OnOffSolenoid::is_on() {
//     return this->state;
// }

// bool OnOffSolenoid::is_max_on() {
//     return this->state && !this->holding;
// }