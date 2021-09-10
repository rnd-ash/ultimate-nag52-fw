#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <pins.h>
#include <Arduino.h>

/**
 * Number of teeth on the N2 carrier gear
 */
#define N2_PULSES_PER_REV 60

/**
 * Number of teeth on the N3 carrier gear
 */
#define N3_PULSES_PER_REV 60

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
    uint16_t read_n2_rpm();

    /**
     * Reads the speed sensor of the N3 input shaft (At front input carrier gear)
     * This reports a speed in gears N,2,3,4,R1,R2
     * 
     * IMPORTANT: This reading is NOT normalized or averaged.
     * It is the callers responsibility to remove any outliers which may
     * result due to signal noise or a faulty speed sensor.
     */
    uint16_t read_n3_rpm();

    uint16_t read_atf_temp();

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