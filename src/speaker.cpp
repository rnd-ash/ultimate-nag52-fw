#include "speaker.h"
#include "board_config.h"
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
        .hpoint = 0,
        .flags = {
            .output_invert = 0u
        }
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

#define PULSE_LONG  300
#define PULSE_SHORT 150

void Speaker::send_note(uint32_t freq, ToneLength tone) {
    int length = PULSE_SHORT;
    if (ToneLength::Long == tone) {
        length = PULSE_LONG;
    }
    this->set_freq(freq);
    vTaskDelay(length/portTICK_PERIOD_MS);
    this->set_freq(0);
    vTaskDelay(100/portTICK_PERIOD_MS);
}


void Speaker::post(SPEAKER_POST_CODE code) {
    switch (code) {
        case SPEAKER_POST_CODE::INIT_OK:
            this->send_note(1500, ToneLength::Short);
            this->send_note(1500, ToneLength::Short);
            break;
        case SPEAKER_POST_CODE::EEPROM_FAIL:
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Long);
            break;
        case SPEAKER_POST_CODE::CAN_FAIL:
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Long);
            break;
        case SPEAKER_POST_CODE::SOLENOID_FAIL:
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Long);
            break;
        case SPEAKER_POST_CODE::SENSOR_FAIL:
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Short);
            break;
        case SPEAKER_POST_CODE::CONTROLLER_FAIL:
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Long);
            break;
        case SPEAKER_POST_CODE::EFUSE_NOT_SET:
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Short);
            break;
        case SPEAKER_POST_CODE::CONFIGURATION_MISMATCH:
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Short);
            break;            
        case SPEAKER_POST_CODE::CALIBRATION_FAIL:
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Short);
            this->send_note(500, ToneLength::Long);
            this->send_note(500, ToneLength::Long);
            break;            
        default:
            break;
    }
}

Speaker* spkr = nullptr;