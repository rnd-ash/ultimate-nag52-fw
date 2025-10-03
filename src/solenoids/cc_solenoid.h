#ifndef __CONSTANT_CURRENT_SOLENOID_H_
#define __CONSTANT_CURRENT_SOLENOID_H_

#include "pwm_solenoid.h"
#include "driver/gptimer.h"
#include "firstorder_average.h"

extern float mpc_sol_trim_factor;

class ConstantCurrentSolenoid : public PwmSolenoid {
public:
    explicit ConstantCurrentSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms);
    void __write_pwm(float vref_compensation, float temperature_factor, bool stop_compensation);
    void set_current_target(uint16_t target_ma);
    void update_when_reading(uint16_t battery);
    uint16_t get_current_target();
    float get_trim();
private:
    float internal_trim_factor = 0.0;
    uint16_t saved_current_target = 0;
    uint16_t current_target = 0;
    bool correct_cycle = false;
};

#endif