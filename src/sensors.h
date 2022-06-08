/** @file */
#ifndef __SENSORS_H_
#define __SENSORS_H_

#include <stdint.h>

/**
 * @brief RPM Reading structure
 * 
 */
typedef struct {
    /// N2 Raw Pulse count
    uint32_t n2_raw;
    /// N3 Raw pulse count
    uint32_t n3_raw;
    /// Calculated input RPM
    uint32_t calc_rpm;
} RpmReading;

namespace Sensors {
    /**
     * @brief Initializes the sensor subsystem of the gearbox
     * 
     * @return true If initialization OK
     * @return false if initialization failed
     */
    bool init_sensors();

    /**
     * @brief Reads the input RPM of the gearbox
     * 
     * @param dest Destination RPM reading pointer
     * @param check_sanity Should this function compare N2 to N3? This is a sanity
     * check done in gears 2,3 and 4, where N2 and N3 must be roughly the same value (+/-50RPM).
     * If that is not the case, then something is wrong with the gearbox input speed sensors
     * @return true If sensor reading OK
     * @return false If sensor reading failed, or if sanity check failed
     */
    bool read_input_rpm(RpmReading* dest, bool check_sanity);

    /**
     * @brief Reads the PCB supply voltage pin in mV
     * 
     * @param dest Destination pointer to store the voltage in mV
     * @return true 
     * @return false 
     */
    bool read_vbatt(uint16_t* dest);
    bool read_atf_temp(int16_t* dest);
    bool parking_lock_engaged(bool* dest);
}

#endif // __SENSORS_H_