#include "channels.h"
#include <pins.h>

PwmChannel::PwmChannel(ledc_channel_t channel, uint32_t freq, uint8_t pin, uint8_t initial_pwm) {
    this->channel = channel;

    ledc_timer_config_t config_cfg = {
        .speed_mode       = ledc_mode_t::LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&config_cfg);

    ledc_channel_config_t channel_cfg = {
        .gpio_num       = pin,
        .speed_mode     = ledc_mode_t::LEDC_LOW_SPEED_MODE,
        .channel        = channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&channel_cfg);
    this->write_pwm(initial_pwm);
};

void PwmChannel::write_pwm(uint8_t value) {
    ledc_set_duty(ledc_mode_t::LEDC_LOW_SPEED_MODE, this->channel, value);
    ledc_update_duty(ledc_mode_t::LEDC_LOW_SPEED_MODE, this->channel);
    this->duty = value;
}

uint8_t PwmChannel::get_pwm() {
    return this->duty;
};

void PwmChannel::change_freq(uint32_t f) {
    ledc_set_freq(ledc_mode_t::LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, f);
    this->write_pwm(this->duty); // Refresh
}


PwmChannel y3_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_0, 1000, PIN_Y3_PWM, 0);
PwmChannel y4_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_1, 1000, PIN_Y4_PWM, 0);
PwmChannel y5_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_2, 1000, PIN_Y5_PWM, 0);

PwmChannel spc_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_3, 1000, PIN_SPC_PWM, 0);
PwmChannel mpc_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_4, 1000, PIN_MPC_PWM, 0);
PwmChannel tcc_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_5, 1000, PIN_TCC_PWM, 0);
PwmChannel spkr_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_6, 1000, PIN_SPKR, 0);