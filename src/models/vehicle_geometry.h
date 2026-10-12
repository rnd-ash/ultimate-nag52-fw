#ifndef __VEHICLE_GEOMETRY_H
#define __VEHICLE_GEOMETRY_H

#include "nvs/eeprom_config.h"

static inline float mps_per_output_rpm(void) {
    const float fallback = 1.975f / 60.0f / 3.070f;
    if (0u == VEHICLE_CONFIG.diff_ratio || 0u == VEHICLE_CONFIG.wheel_circumference) {
        return fallback;
    }
    return ((float)VEHICLE_CONFIG.wheel_circumference / 1000.0f) / 60.0f /
           ((float)VEHICLE_CONFIG.diff_ratio / 1000.0f);
}

#endif
