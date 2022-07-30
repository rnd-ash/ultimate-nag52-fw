
#ifndef __SOLENOID_H_
#define __SOLENOID_H_

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include <driver/i2s.h>
#include <soc/syscon_reg.h>
#include <driver/adc.h>
#include <esp_event.h>

const static float SOLENOID_VREF = 12000.0f; // 12V Vref for solenoids

#define SOLENOID_CURRENT_AVG_SAMPLES 10

class Solenoid
{
public:
    /**
     * @brief Construct a new Solenoid
     * 
     * @param name Name of the solenoid (Avaliable from 722.6 PDFs)
     * @param pwm_pin GPIO Pin on the ESP32 to use for PWM control of the solenoid
     * @param frequency Default frequency of the PWM signal for the solenoid
     * @param channel LEDC Channel to use for controlling the solenoids PWM signal
     * @param timer Timer to use for controlling the solenoids PWM signal
     * @param read_channel The ADC 1 Channel used for current sense feedback
     */
    Solenoid(const char *name, gpio_num_t pwm_pin, uint32_t frequency, ledc_channel_t channel, ledc_timer_t timer, adc1_channel_t read_channel);


    const char* get_name() {
        return this->name;
    }

    /**
     * @brief Writes a 12-bit PWM duty to the solenoid
     * 
     * @param percent Requested PWM duty (0-4096)
     * @param voltage_compensate (Default true) - Tells the solenoid API
     * weather to take into account the real voltage being supplied to the solenoid
     * in order to maintain a constant current. 
     * 
     * For instance, if the desired PWM is based on a 12.0V voltage, but the actual
     * voltage being supplied to the solenoid is 14.0V, then a ~16% reduction in PWM
     * will be made in order to keep the current being sent to the solenoid the same
     */
    void write_pwm_12_bit(uint16_t pwm_raw, bool voltage_compensate = true);
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
    uint16_t get_pwm_compensated();

    /**
     * @brief Gets the average current consumed by the solenoid
     * over multiple I2S samples
     */
    uint16_t get_current_avg();

    /**
     * @brief Gets the current consumed by the solenoid at the previous I2S sample
     */
    uint16_t get_current();

    /**
     * @brief returns the ADC1 channel being used to read
     * the current of the solenoid
     */
    adc1_channel_t get_adc_channel();

    /**
     * @brief Returns if the solenoid initialized OK
     * 
     * @return true LEDC / Timer initialized OK, and no short-circuit present within the solenoid circuit
     * @return false Something went wrong trying to intialize the solenoid
     */
    bool init_ok() const;

    // Internal functions - Don't touch, handled by I2S thread!
    void __set_adc_reading(uint16_t c);
    void __write_pwm();

    uint16_t diag_get_adc_peak_raw();

    // -- These functions are only accessed by sw_fader class! -- //
private:
    uint32_t default_freq;
    bool ready;
    const char *name;
    ledc_channel_t channel;
    ledc_timer_t timer;
    uint16_t adc_reading_current;
    bool voltage_compensate = true;
    adc1_channel_t adc_channel;
    uint16_t pwm = 0;
    uint16_t pwm_raw = 0;

    // For avg current
    uint16_t adc_avgs[SOLENOID_CURRENT_AVG_SAMPLES];
    uint64_t adc_total;
    uint8_t  adc_sample_idx;
};

#define I2S_LOOP_INTERVAL_CC_ONLY 10
#define I2S_LOOP_INVERVAL_ALL 20

namespace Solenoids {
    /**
     * @brief Tries to initialize all the solenoids on the transmission (MPC,SPC,TCC,Y3,Y4,Y5)
     * 
     * @return true All solenoids initialized OK
     * @return false A solenoid failed to initialize
     */
    bool init_all_solenoids();
    uint16_t get_solenoid_voltage();
    void toggle_all_solenoid_current_monitoring(bool enable);
    bool is_monitoring_all_solenoids();
}

extern Solenoid *sol_y3;
extern Solenoid *sol_y4;
extern Solenoid *sol_y5;

extern Solenoid *sol_mpc;
extern Solenoid *sol_spc;
extern Solenoid *sol_tcc;

extern float resistance_spc;
extern float resistance_mpc;
extern bool temp_cal;
extern int16_t temp_at_test;

#define DIAG_DMA_BUFFER_LEN 500
#define I2S_DMA_BUF_LEN 1024
extern uint16_t* buf;

#endif // __SOLENOID_H_