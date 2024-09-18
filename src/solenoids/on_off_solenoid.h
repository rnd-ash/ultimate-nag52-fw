#ifndef ON_OFF_SOLENOID_H
#define ON_OFF_SOLENOID_H

#include "pwm_solenoid.h"
#include "driver/gptimer.h"

class OnOffSolenoid : public PwmSolenoid {
public:
    explicit OnOffSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint32_t on_time_ms, uint16_t hold_pwm, uint16_t phase_duration_ms);
    void __write_pwm(float vref_compensation, float temperature_factor);
    void on(void);
    void off(void);
    /* unused */
    // bool is_on(void);
    // bool is_max_on(void);

private:
    uint32_t on_time_ms = 0u;
    uint32_t target_on_time;
    uint16_t target_hold_pwm;
    bool state = false;
    bool holding = false;
};

#endif