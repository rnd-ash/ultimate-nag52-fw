
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

const static float solenoid_vref = 12000.0f; // 12V Vref for solenoids

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
     */
    Solenoid(const char *name, gpio_num_t pwm_pin, uint32_t frequency, ledc_channel_t channel, ledc_timer_t timer);

    /**
     * @brief Writes a raw 12-bit PWM duty to the solenoid
     * 
     * @param percent Raw PWM duty (0-4096)
     */
    void write_pwm_12_bit(uint16_t pwm_raw);

    /**
     * @brief Writes a raw 12-bit PWM duty to the solenoid, but also takes
     * the current voltage into account and adjusts PWM based on the current voltage to maintain
     * a constant current.
     * 
     * NOTE: At this time, the reference voltage used is 12000mV (12.0V)
     * 
     * @param duty Raw PWM duty (0-4096)
     * @param curr_v_mv The current supply voltage to the TCM, in mV
     */
    void write_pwm_12bit_with_voltage(uint16_t duty, uint16_t curr_v_mv);
    /**
     * @brief Returns the current PWM duty of the solenoid
     * 
     * @return uint16_t current 12-bit PWM duty of the solenoid
     */
    uint16_t get_pwm();

    /**
     * @brief Gets an estimate of the current being used by the solenoid
     * 
     * This function updates approximatly every 200ms for each solenoid, so
     * after requesting a PWM change, the current reading might not change instantly
     * 
     * @return uint16_t Current estimate of the solenoid, in milliamps (mA)
     */
    uint16_t get_current_estimate();

    /**
     * @brief Returns if the solenoid initialized OK
     * 
     * @return true LEDC / Timer initialized OK, and no short-circuit present within the solenoid circuit
     * @return false Something went wrong trying to intialize the solenoid
     */
    bool init_ok() const;

    /**
     * @brief Returns the solenoids base-line voltage reference as seen by the current monitoring circuit for the solenoid
     * 
     * NOTE: This function should NOT be used, it is only for development purposes!
     * 
     * @return uint16_t Voltage reference point of the solenoid current measuring circuit, in mV
     */
    uint16_t get_vref() const;
    // Internal functions - Don't touch, handled by I2S thread!
    void __set_current_internal(uint16_t c);
    void __set_vref(uint16_t ref);

    // -- These functions are only accessed by sw_fader class! -- //
private:
    uint32_t default_freq;
    bool ready;
    const char *name;
    uint16_t vref;
    bool vref_calibrated;
    ledc_channel_t channel;
    ledc_timer_t timer;
    portMUX_TYPE adc_reading_mutex;
    volatile uint16_t adc_reading;
    uint16_t pwm = 0;
};

/**
 * @brief Tries to initialize all the solenoids on the transmission (MPC,SPC,TCC,Y3,Y4,Y5)
 * 
 * @return true All solenoids initialized OK
 * @return false A solenoid failed to initialize
 */
bool init_all_solenoids();

extern Solenoid *sol_y3;
extern Solenoid *sol_y4;
extern Solenoid *sol_y5;

extern Solenoid *sol_mpc;
extern Solenoid *sol_spc;
extern Solenoid *sol_tcc;

enum PwmSolenoid {
    PWM_SPC = 0,
    PWM_MPC = 1
};

extern Solenoid* pressure_pwm_solenoids[2];

#endif // __SOLENOID_H_