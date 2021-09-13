#ifndef PWM_CHANNELS_H_
#define PWM_CHANNELS_H_

#include <stdint.h>
#include <driver/ledc.h>

class PwmChannel {
public:
    PwmChannel(ledc_channel_t channel, uint32_t freq, uint8_t pin, uint8_t initial_pwm);
    void write_pwm(uint8_t value);
    uint8_t get_pwm();
    void change_freq(uint32_t f);
private:
    ledc_channel_t channel;
    uint8_t duty = 0;
};

extern PwmChannel y3_pwm;
extern PwmChannel y4_pwm;
extern PwmChannel y5_pwm;

extern PwmChannel spc_pwm;
extern PwmChannel mpc_pwm;
extern PwmChannel tcc_pwm;

extern PwmChannel spkr_pwm;

#endif