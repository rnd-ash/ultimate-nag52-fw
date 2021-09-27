#include "channels.h"
#include <pins.h>

#include <esp_log.h>

#define FADE_STEP_SIZE 10

PwmChannel::PwmChannel(ledc_channel_t channel, uint32_t freq, uint8_t pin, uint8_t initial_pwm, ledc_timer_t timer) {
    this->channel = channel;
    this->timer = timer;
    ledc_timer_config_t config_cfg = {
        .speed_mode       = ledc_mode_t::LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = timer,
        .freq_hz          = freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&config_cfg);

    ledc_channel_config_t channel_cfg = {
        .gpio_num       = pin,
        .speed_mode     = ledc_mode_t::LEDC_LOW_SPEED_MODE,
        .channel        = channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = timer,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&channel_cfg);
    this->write_pwm(initial_pwm);
};

void PwmChannel::write_pwm(uint8_t value) {
    ledc_set_duty(ledc_mode_t::LEDC_LOW_SPEED_MODE, this->channel, (uint8_t)value);
    ledc_update_duty(ledc_mode_t::LEDC_LOW_SPEED_MODE, this->channel);
    this->__duty = (int)value*10;
}

uint8_t PwmChannel::get_pwm() {
    return (uint8_t)(this->__duty/10);
};

void PwmChannel::change_freq(uint8_t f) {
    ledc_set_freq(ledc_mode_t::LEDC_LOW_SPEED_MODE, this->timer, f);
    this->write_pwm(this->__duty); // Refresh
    this->__duty = f*10;
    this->__step = 0;
    this->__target_duty = f*10;
}

void PwmChannel::fade_pwm(uint8_t dest_pwm, uint32_t time_ms) {
    if (this->__duty/10 == dest_pwm) { return; }
    if (time_ms <= FADE_STEP_SIZE) {
        this->change_freq(dest_pwm); // Too quick!
    } else {
        int steps = time_ms/FADE_STEP_SIZE;
        if (dest_pwm > this->__duty) {
            // increase duty
            this->__step = ((int)(dest_pwm*10) - this->__duty) / steps;
        } else {
            // decrease duty
            this->__step = (this->__duty - (int)(dest_pwm*10))*-1 / steps;
        }
        this->__target_duty = dest_pwm*10;
    }
}


PwmChannel y3_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_0, 1000, PIN_Y3_PWM, 0, LEDC_TIMER_0);
PwmChannel y4_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_1, 1000, PIN_Y4_PWM, 0, LEDC_TIMER_0);
PwmChannel y5_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_2, 1000, PIN_Y5_PWM, 0, LEDC_TIMER_0);

PwmChannel spc_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_3, 1000, PIN_SPC_PWM, 0, LEDC_TIMER_1);
PwmChannel mpc_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_4, 1000, PIN_MPC_PWM, 0, LEDC_TIMER_1);
PwmChannel tcc_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_5, 1000, PIN_TCC_PWM, 0, LEDC_TIMER_2);
PwmChannel spkr_pwm = PwmChannel(ledc_channel_t::LEDC_CHANNEL_6, 1000, PIN_SPKR, 0, LEDC_TIMER_3);


uint16_t tmp = 0;
void fade_channel(PwmChannel* c) {
    if (c->__step != 0) {
        tmp = c->__duty;
        c->write_pwm((c->__duty+c->__step)/10);
        c->__duty = tmp+c->__step;
        if (((c->__duty >= c->__target_duty) && c->__step > 0) || ((c->__duty <= c->__target_duty) && c->__step < 0)) {
            c->__step = 0;
            c->write_pwm((c->__target_duty)/10);
        }
    }
}

[[noreturn]] void pwm_solenoid_fader_thread(void* args) {
    while(true) {
        fade_channel(&tcc_pwm);
        fade_channel(&mpc_pwm);
        fade_channel(&spc_pwm);
        vTaskDelay(FADE_STEP_SIZE);
    }
}