/** @file */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "esp_err.h"

struct SensorDataRaw {
    uint16_t rpm_n2;
    uint16_t rpm_n3;
    uint16_t rpm_out;
    uint16_t battery_mv;
    int atf_temp_c;
    uint8_t parking_lock;
};

namespace Sensors {
    /**
     * @brief Initializes the sensor subsystem of the gearbox
     * 
     * @return true If initialization OK
     * @return false if initialization failed
     */
    esp_err_t init_sensors(void);
    void update(SensorDataRaw* dest);
    bool using_dedicated_output_rpm();
}

#endif // SENSORS_H