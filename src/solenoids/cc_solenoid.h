#ifndef __CONSTANT_CURRENT_SOLENOID_H_
#define __CONSTANT_CURRENT_SOLENOID_H_

#include "pwm_solenoid.h"
#include "driver/gptimer.h"

class ConstantCurrentSolenoid : public PwmSolenoid {
public:
    explicit ConstantCurrentSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms);
    void __write_pwm(float vref_compensation, float temperature_factor);
    void set_current_target(uint16_t target_ma);
    uint16_t get_current_target();
    float get_trim();
private:
    float internal_trim_factor = 0.0;
    uint16_t current_target = 0;
    uint16_t prev_step_current_target = 0;
    uint16_t prev_step_max = 0;
    uint16_t prev_step_v = 0;
    uint8_t counter = 0;
};

#endif