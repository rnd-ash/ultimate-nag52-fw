#ifndef __VEHICLE_GEOMETRY_H
#define __VEHICLE_GEOMETRY_H

#include "nvs/eeprom_config.h"

/**
 * @brief Metres per second of road speed per RPM of the output shaft.
 *
 * The one place the wheel circumference and final drive are turned into a road
 * speed, so that everything reporting in SI agrees and there is a single place
 * to be wrong. Parameters and reported metrics are stated in SI wherever it is
 * reasonable, and this is what makes that possible.
 *
 * Accuracy barely matters for what uses it. A properly plus-sized wheel changes
 * the circumference by about 1 % (195/65R15 and 225/45R17 are within 0.1 % of
 * each other) and a 20 inch wheel nobody would fit to a W210 by 8 %. What DOES
 * matter is not silently returning zero, which is what the guards below are for:
 * the factory default config is 2850 mm on a 1.000 final drive, i.e. a
 * placeholder, and an unconfigured TCU reporting a jerk of 0 would look like a
 * perfect shift rather than an unconfigured TCU.
 */
static inline float mps_per_output_rpm(void) {
    // Placeholder-config fallback: a 1.975 m circumference on a 3.07 final drive,
    // which is an ordinary saloon. Wrong for any given car, but the right order
    // of magnitude, where 0 is not.
    const float FALLBACK = 1.975f / 60.0f / 3.070f;
    if (0u == VEHICLE_CONFIG.diff_ratio || 0u == VEHICLE_CONFIG.wheel_circumference) {
        return FALLBACK;
    }
    // circumference is mm, diff_ratio is x1000
    return ((float)VEHICLE_CONFIG.wheel_circumference / 1000.0f) / 60.0f /
           ((float)VEHICLE_CONFIG.diff_ratio / 1000.0f);
}

#endif
