/** @file */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief RPM Reading structure
 * 
 */
struct RpmReading{
    /// N2 Raw Pulse count
    uint32_t n2_raw;
    /// N3 Raw pulse count
    uint32_t n3_raw;
    /// Calculated input RPM
    uint32_t calc_rpm;
};

namespace Sensors {
    /**
     * @brief Initializes the sensor subsystem of the gearbox
     * 
     * @return true If initialization OK
     * @return false if initialization failed
     */
    esp_err_t init_sensors(void);
    
    /**
     * @brief Sets the Ratio of Gear2/Gear1. This is used when doing RPM calculations
     * for the input shaft speed
    */
    void set_ratio_2_1(float r);
    float get_ratio_2_1();

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
    esp_err_t read_input_rpm(RpmReading* dest, bool check_sanity);

    /**
     * @brief ONLY FOR BOARD V1.3+! Reads output RPM of the gearbox when the GPIO pin is configured for RPM reading
     * 
     * @param dest Destination to store the output RPM
     * @return true RPM Reading OK
     * @return false Error reading RPM
     */
    esp_err_t read_output_rpm(uint16_t* dest);

    /**
     * @brief Reads the PCB supply voltage pin in mV
     * 
     * @param dest Destination pointer to store the voltage in mV
     * @return true 
     * @return false 
     */
    esp_err_t read_vbatt(uint16_t* dest);

    /**
     * @brief Reads the ATF temp from the TFT sensor in degrees C.
     * NOTE: If parking lock is engaged, then motor coolant temperature provided
     * by the CAN layer is instead used. This makes the most sense as motor coolant
     * and transmission oil run in the same cooler.
     * 
     * @param dest 
     * @return ESP_OK - ATF temp reading OK
     * ESP_ERR_INVALID_STATE - Parking lock engaged
     */
    esp_err_t read_atf_temp(int16_t* dest);

    /**
     * @brief Reads the ATF temp from the TFT sensor in 1/10th degrees C.
     * NOTE: If parking lock is engaged, then motor coolant temperature provided
     * by the CAN layer is instead used. This makes the most sense as motor coolant
     * and transmission oil run in the same cooler.
     * 
     * @param dest ATF temperature in degrees C, multiplied by 10 (EG: 12C => 120)
     * @return ESP_OK - ATF temp reading OK
     * ESP_ERR_INVALID_STATE - Parking lock engaged
     */
    esp_err_t read_atf_temp_fine(int16_t* dest);

    esp_err_t parking_lock_engaged(bool* dest);
    void set_motor_temperature(int16_t celcius);
}

#endif // SENSORS_H