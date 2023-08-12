#ifndef __PWM_SOLENOID_H_
#define __PWM_SOLENOID_H_

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "hal/adc_types.h"
#include <esp_event.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "tcu_maths.h"
#include "moving_average.h"

extern uint16_t voltage;
extern uint16_t min_adc_v_reading;

extern const ledc_timer_t SOLENOID_TIMER;
extern const ledc_timer_config_t SOLENOID_TIMER_CFG;

class PwmSolenoid
{
public:
    /**
     * @brief Construct a new Solenoid
     * 
     * @param name Name of the solenoid (Avaliable from 722.6 PDFs)
     * @param pwm_pin GPIO Pin on the ESP32 to use for PWM control of the solenoid
     * @param frequency Default frequency of the PWM signal for the solenoid
     * @param channel LEDC Channel to use for controlling the solenoids PWM signal
     * @param read_channel The ADC 1 Channel used for current sense feedback
     * @param current_samples The number of samples from I2S for current measuring. It is assumed that each sample is ~2ms snapshot
     */
    PwmSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint8_t current_samples);


    const char* get_name() {
        return this->name;
    }

    /**
     * @brief Returns the raw PWM that was requested by write_pwm_12_bit
     * 
     * @return The raw PWM duty that was requested
     */
    uint16_t get_pwm_raw();

    /**
     * @brief returns the actual PWM that is being written to the solenoid
     * 
     * This can differ from get_pwm_raw when voltage_compensate is being used
     * by write_pwm_12_bit
     * 
     * @return The current PWM being written to the solenoid
     */
    uint16_t get_pwm_compensated() const;

    /**
     * @brief Gets the current consumed by the solenoid at the previous I2S sample
     */
    uint16_t get_current() const;

    /**
     * @brief returns the ADC1 channel being used to read
     * the current of the solenoid
     */
    adc_channel_t get_adc_channel() const;

    /**
     * @brief Returns if the solenoid initialized OK
     * 
     * @return true LEDC / Timer initialized OK, and no short-circuit present within the solenoid circuit
     * @return false Something went wrong trying to intialize the solenoid
     */
    esp_err_t init_ok() const;

    // Internal functions - Don't touch, handled by I2S thread!
    void __set_adc_reading(uint16_t c);

    // -- These functions are only accessed by sw_fader class! -- //
protected:
    MovingAverage* current_avg_samples;
    esp_err_t ready;
    const char *name;
    ledc_channel_t channel;
    bool voltage_compensate = true;
    adc_channel_t adc_channel;
    uint16_t pwm = 0;
    uint16_t pwm_raw = 0;

    // For avg current
    uint16_t adc_now_read = 0;
};

#endif