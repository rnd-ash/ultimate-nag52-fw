#ifndef __INRUSH_SOLENOID_H_
#define __INRUSH_SOLENOID_H_

#include "pwm_solenoid.h"
#include "driver/gptimer.h"

class InrushControlSolenoid : public PwmSolenoid {
public:
    explicit InrushControlSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms);
    void __write_pwm(float vref_compensation, float temperature_factor);
    uint32_t on_timer_interrupt();
    void set_duty(uint16_t duty);
    void pre_current_test() override;
    void post_current_test() override;
private:
    uint32_t period_duration_us = 0;
    uint16_t vref_compensation = 100; // %
    uint16_t temp_compensation = 100; // %
    // 0 - Inrush
    // 1 - Hold
    // 2 - Off
    uint8_t phase_id = 2; // So we start at 0 again
    uint32_t period_on_time = 0;
    uint32_t period_off_time = 0;
    uint16_t target_hold_current = 0;
    uint16_t calc_hold_pwm = 1024;
    uint32_t inrush_time = 20000; // at 12V and 25C
    uint32_t hold_time = 0; 
    gptimer_handle_t timer;
    bool off = false;
};

#endif