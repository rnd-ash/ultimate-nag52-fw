#ifndef __CLUTCH_SPEED_H_
#define __CLUTCH_SPEED_H_

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

struct SpeedSensors {
    uint16_t n2;
    uint16_t n3;
    uint16_t turbine;
    uint16_t output;
};

namespace ClutchSpeedModel {
    ShiftClutchData get_shifting_clutch_speeds(const SpeedSensors speeds, const ProfileGearChange req, const GearRatioInfo* ratios);
    ClutchSpeeds get_clutch_speeds_debug(
        const SpeedSensors speeds,
        const GearboxGear last_motion_gear,
        const GearboxGear actual,
        const GearboxGear target,
        const GearRatioInfo* ratios
    );
}

#endif