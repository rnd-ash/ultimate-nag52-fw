#include "speaker.h"
#include "pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

Speaker::Speaker(gpio_num_t pin) {
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = ledc_mode_t::LEDC_LOW_SPEED_MODE, // Low speed timer mode
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_3,
        .freq_hz = 100,
        .clk_cfg = LEDC_AUTO_CLK
    };
    // Set the timer configuration
    ledc_timer_config(&timer_cfg);

    // Set PWM channel configuration
    const ledc_channel_config_t channel_cfg = {
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

void Speaker::send_note(uint32_t freq, uint32_t play_time_ms, uint32_t total_time_ms) {
    this->set_freq(freq);
    vTaskDelay(play_time_ms/portTICK_PERIOD_MS);
    this->set_freq(0);
    vTaskDelay((total_time_ms-play_time_ms)/portTICK_PERIOD_MS);
}

#define PULSE_LONG  500
#define PULSE_SHORT 250
#define TOTAL_PULSE 550

void Speaker::post(SPEAKER_POST_CODE code) {
    switch (code) {
        case SPEAKER_POST_CODE::INIT_OK:
            this->send_note(1500, 150, 200);
            this->send_note(1500, 150, 200);
            break;
        case SPEAKER_POST_CODE::EEPROM_FAIL:
            this->send_note(500, 300, 500);
            this->send_note(500, 300, 500);
            this->send_note(500, 300, 500);
            this->send_note(500, 300, 500);
            break;
        case SPEAKER_POST_CODE::CAN_FAIL:
            this->send_note(500, 300, 500);
            this->send_note(500, 300, 500);
            this->send_note(500, 150, 200);
            this->send_note(500, 300, 500);
            break;
        case SPEAKER_POST_CODE::SOLENOID_FAIL:
            this->send_note(500, 300, 500);
            this->send_note(500, 150, 200);
            this->send_note(500, 300, 500);
            this->send_note(500, 300, 500);
            break;
        case SPEAKER_POST_CODE::SENSOR_FAIL:
            this->send_note(500, 300, 500);
            this->send_note(500, 150, 200);
            this->send_note(500, 150, 200);
            this->send_note(500, 150, 200);
            break;
        case SPEAKER_POST_CODE::CONTROLLER_FAIL:
            this->send_note(500, 300, 500);
            this->send_note(500, 150, 200);
            this->send_note(500, 150, 200);
            this->send_note(500, 300, 500);
            break;
        default:
            break;
    }
}



Speaker spkr = Speaker(PIN_SPKR);