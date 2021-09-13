#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <pins.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"

/**
 * Number of teeth on the N2 carrier gear
 */
#define N2_PULSES_PER_REV 60

/**
 * Number of teeth on the N3 carrier gear
 */
#define N3_PULSES_PER_REV 60


/** 
 * ADC1 channels for solenoid current monitoring
 */
#define ADC_CHANNEL_Y3 adc1_channel_t::ADC1_CHANNEL_4
#define ADC_CHANNEL_Y4 adc1_channel_t::ADC1_CHANNEL_5
#define ADC_CHANNEL_Y5 adc2_channel_t::ADC2_CHANNEL_9

#define ADC_CHANNEL_SPC adc2_channel_t::ADC2_CHANNEL_7
#define ADC_CHANNEL_MPC adc2_channel_t::ADC2_CHANNEL_8
#define ADC_CHANNEL_TCC adc2_channel_t::ADC2_CHANNEL_6

#define ADC_CHANNEL_VSENSE adc1_channel_t::ADC1_CHANNEL_0
#define ADC_CHANNEL_ATF adc1_channel_t::ADC1_CHANNEL_3


enum class ADC_Reading {
    Y3,
    Y4,
    Y5,
    MPC,
    SPC,
    TCC,
    ATF,
    V_SENSE
};

/**
 * Solenoids on the gearbox
 */
enum class Solenoid {
    // Y3 shift solenoid (Controls 1-2/4-5 shifting)
    Y3 = 0,
    // Y4 shift solenoid (Controls 3-4 shifting)
    Y4 = 1,
    // Y5 shift solenoid (Controls 2-3 shifting)
    Y5 = 2,
    // Shift pressure solenoid (Controls clutch pack pressure during shifting)
    SPC = 3,
    // Modulating pressure solenoid (Controls overall pressure to the gearbox)
    MPC = 4,
    // Torque converter clutch solenoid (Controls clutch application in the torque converter)
    TCC = 5
};

namespace Sensors {
    /**
     * Run this once on startup to configure all the sensor pins
     */
    void configure_sensor_pins();

    /**
     * Returns the battery voltage sensed in millivolts (mV)
     * 
     * Example: 1430 = 14.3V
     */
    uint16_t read_vbatt();


    /**
     * Reads the speed sensor of the N2 input shaft (At front sun gear).
     * This reports a speed in gears N,1,2,3,4,5
     * 
     * IMPORTANT: This reading is NOT normalized or averaged.
     * It is the callers responsibility to remove any outliers which may
     * result due to signal noise or a faulty speed sensor.
     */
    uint32_t read_n2_rpm();

    /**
     * Reads the speed sensor of the N3 input shaft (At front input carrier gear)
     * This reports a speed in gears N,2,3,4,R1,R2
     * 
     * IMPORTANT: This reading is NOT normalized or averaged.
     * It is the callers responsibility to remove any outliers which may
     * result due to signal noise or a faulty speed sensor.
     */
    uint32_t read_n3_rpm();

    /**
     * Reads the ATF temperature from the gearbox.
     * 
     * If read_park_lock returns true, this function CANNOT be used
     * as it'll give a false reading.
     */
    int16_t read_atf_temp();

    /**
     * Reads the park lock status within the gearbox
     * 
     * If this function returns true, then reading ATF temp should not occur
     * and instead the gearbox should substitute engine temp for gearbox ATF temp.
     * When false, it means the gearbox is either in R or D, starting the engine
     * is not allowed, and the function read_atf_temp can be used as normal
     */
    bool read_park_lock();

    /**
     * Returns the current amperage being read from the solenoid circuit.
     * 0 = 0A
     * 6000 = 6A (Max read)
     * 
     * All solenoids should never exceed 5A, and normal operation should never see more than 2A.
     * If over current is detected, the caller must disable the solenoid as there might be a short!
     */
    uint32_t read_solenoid_current(Solenoid sol);
}

#endif