#ifndef __ON_OFF_SOLENOID_H_
#define __ON_OFF_SOLENOID_H_

#include "pwm_solenoid.h"
#include "driver/gptimer.h"

class OnOffSolenoid : public PwmSolenoid {
public:
    explicit OnOffSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint32_t on_time_ms, uint16_t hold_pwm, uint16_t phase_duration_ms);
    void __write_pwm(float vref_compensation, float temperature_factor);
    void on();
    void off();
    bool is_on();
    bool is_max_on();

private:
    uint32_t on_time_ms;
    uint32_t target_on_time;
    uint16_t target_hold_pwm;
    bool state = false;
    bool holding = false;
};

#endif