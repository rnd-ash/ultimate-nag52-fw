
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
     * Constructor params
     * name - Name of solenoid (Human readable)
     * pwm_pin - PWM pin to gate of MOSFET
     * frequency - Default frequency of PWM for solenoid
     * channel - LEDC channel designated to the solenoid
     * timer - HW timer for controlling PWM
     */
    Solenoid(const char *name, gpio_num_t pwm_pin, uint32_t frequency, ledc_channel_t channel, ledc_timer_t timer);
    void write_pwm_percent(uint16_t percent); // Write PMW percentage (0 - 0%, 1000 = 100%)
    void write_pwm_percent_with_voltage(uint16_t percent, uint16_t curr_v_mv); // Write PWM percentage with voltage correction
    void write_pwm_12_bit(uint16_t pwm_raw); // Write raw 12bit PWM signal to solenoid
    void write_pwm_12bit_with_voltage(uint16_t duty, uint16_t curr_v_mv); // Write 12bit PWM with voltage correction (Mainly used by torque converter solenoid)
    uint16_t get_pwm(); // Returns PWM signal of solenoid
    uint16_t get_current_estimate(); // Returns current estimate of the solenoid
    bool init_ok() const; // Did the solenoid initialize OK?
    uint16_t get_vref() const; // Gets the solenoids' vref's calibrated value
    // Internal functions - Don't touch, handled by I2S thread!
    void __set_current_internal(uint16_t c);
    void __set_vref(uint16_t ref);
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

bool init_all_solenoids();

extern Solenoid *sol_y3;
extern Solenoid *sol_y4;
extern Solenoid *sol_y5;

extern Solenoid *sol_mpc;
extern Solenoid *sol_spc;
extern Solenoid *sol_tcc;

#endif // __SOLENOID_H_