#ifndef PWM_CHANNELS_H_
#define PWM_CHANNELS_H_

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>

class PwmChannel {
public:
    PwmChannel(ledc_channel_t channel, uint32_t freq, uint8_t pin, uint8_t initial_pwm, ledc_timer_t timer);
    void write_pwm(uint8_t value);
    uint8_t get_pwm();
    void change_freq(uint8_t f);
    void fade_pwm(uint8_t dest_pwm, uint32_t time_ms);
    int __duty = 0;
    int __target_duty = 0;
    int __step = 0;
    bool down = false;
private:
    ledc_channel_t channel;
    ledc_timer_t timer;
};

extern PwmChannel y3_pwm;
extern PwmChannel y4_pwm;
extern PwmChannel y5_pwm;

extern PwmChannel spc_pwm;
extern PwmChannel mpc_pwm;
extern PwmChannel tcc_pwm;

extern PwmChannel spkr_pwm;

[[noreturn]] void pwm_solenoid_fader_thread(void* args);

#endif