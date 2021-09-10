#ifndef PWM_CHANNELS_H_
#define PWM_CHANNELS_H_

#include <stdint.h>

class PwmChannel {
public:
    PwmChannel(uint8_t channel_id, uint32_t freq, uint8_t pin, uint8_t initial_pwm);
    void write_pwm(uint8_t value);
    uint8_t get_pwm();
private:
    uint8_t channel_id;
};

extern PwmChannel y3_pwm;
extern PwmChannel y4_pwm;
extern PwmChannel y5_pwm;

extern PwmChannel spc_pwm;
extern PwmChannel mpc_pwm;
extern PwmChannel tcc_pwm;

extern PwmChannel spkr_pwm;

#endif