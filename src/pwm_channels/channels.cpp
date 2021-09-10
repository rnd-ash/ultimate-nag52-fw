#include "channels.h"
#include <pins.h>
#include <Arduino.h>



PwmChannel::PwmChannel(uint8_t channel_id, uint32_t freq, uint8_t pin, uint8_t initial_pwm) {
    ledcSetup(channel_id, freq, 8); // 8bit resolution - TODO change this to 12 bit!
    ledcAttachPin(pin, channel_id);
    ledcWrite(channel_id, initial_pwm);
    this->channel_id = channel_id;
};

void PwmChannel::write_pwm(uint8_t value) {
    ledcWrite(this->channel_id, value);
}

uint8_t PwmChannel::get_pwm() {
    return (uint8_t)ledcRead(this->channel_id);
};


PwmChannel y3_pwm = PwmChannel(1, 1000, PIN_Y3_PWM, 0);
PwmChannel y4_pwm = PwmChannel(2, 1000, PIN_Y4_PWM, 0);
PwmChannel y5_pwm = PwmChannel(3, 1000, PIN_Y5_PWM, 0);

PwmChannel spc_pwm = PwmChannel(4, 1000, PIN_SPC_PWM, 0);
PwmChannel mpc_pwm = PwmChannel(5, 1000, PIN_MPC_PWM, 0);
PwmChannel tcc_pwm = PwmChannel(6, 1000, PIN_TCC_PWM, 0);
