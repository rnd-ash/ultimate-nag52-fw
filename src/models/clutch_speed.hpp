#ifndef CLUTCH_SPEED_H
#define CLUTCH_SPEED_H

#include <stdint.h>
#include "common_structs.h"
#include "sensors.h"

struct __attribute__ ((packed)) ClutchSpeeds {
    int16_t k1;
    int16_t k2;
    int16_t k3;
    int16_t b1;
    int16_t b2;
    int16_t b3;
};

namespace ClutchSpeedModel {
    ShiftClutchData get_shifting_clutch_speeds(const uint16_t output_speed, const RpmReading input, const ProfileGearChange req, const GearRatioInfo* ratios);
    ClutchSpeeds get_clutch_speeds_debug(
        const uint16_t output_speed, 
        const RpmReading input, 
        const GearboxGear last_motion_gear,
        const GearboxGear actual,
        const GearboxGear target,
        const GearRatioInfo* ratios
    );
}

#endif