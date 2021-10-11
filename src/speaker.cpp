#include "speaker.h"
#include "pins.h"
#include "driver/ledc.h"

Speaker::Speaker(gpio_num_t pin) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = ledc_mode_t::LEDC_LOW_SPEED_MODE, // Low speed timer mode
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_3,
        .freq_hz = 100,
        .clk_cfg = LEDC_AUTO_CLK
    };
    // Set the timer configuration
    ledc_timer_config(&timer_cfg);

    // Set PWM channel configuration
    ledc_channel_config_t channel_cfg = {
        .gpio_num = pin,
        .speed_mode = ledc_mode_t::LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_6,
        .intr_type = LEDC_INTR_DISABLE, // Disable fade interrupt
        .timer_sel = LEDC_TIMER_3,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&channel_cfg);
}

void Speaker::set_freq(uint32_t freq) {
    if (freq != 0) {
        ledc_set_freq(ledc_mode_t::LEDC_LOW_SPEED_MODE, LEDC_TIMER_3, freq);
        ledc_set_duty(ledc_mode_t::LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, 128);
        ledc_update_duty(ledc_mode_t::LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
    } else {
        ledc_set_duty(ledc_mode_t::LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, 0);
        ledc_update_duty(ledc_mode_t::LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
    }
}

Speaker spkr = Speaker(PIN_SPKR);