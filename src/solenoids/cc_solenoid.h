#ifndef __CONSTANT_CURRENT_SOLENOID_H_
#define __CONSTANT_CURRENT_SOLENOID_H_

#include "pwm_solenoid.h"
#include "driver/gptimer.h"

extern float mpc_sol_trim_factor;

class ConstantCurrentSolenoid : public PwmSolenoid {
public:
    explicit ConstantCurrentSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms, bool is_mpc);
    void __write_pwm(float vref_compensation, float temperature_factor, bool stop_compensation);
    void set_current_target(uint16_t target_ma);
    void set_target_current_when_reading();
    uint16_t get_current_target();
    float get_trim();
private:
    uint16_t old_current_targets[10];
    uint8_t current_target_idx = 0;
    float internal_trim_factor = 0.0;
    uint16_t current_target = 0;
    uint16_t current_target_at_report_time = 0;
    uint8_t c = 0;
    bool use_global_cc;
};

#endif