#ifndef __ADAPT_MAP_H__
#define __ADAPT_MAP_H__

#include <stdint.h>
#include <gearbox_config.h>
#include "common_structs.h"

// Adaptation is only performed if engine is under 2500RPM
// and Engine output torque is as defined:
#ifdef LARGE_NAG
    #define ADAPT_TORQUE_LIMIT 250
#else
    #define ADAPT_TORQUE_LIMIT 150
#endif

#define ADAPT_RPM_LIMIT 2500
#define ADAPT_TEMP_THRESH 60
#define ADAPT_TEMP_LIMIT 100

struct AdaptationData {
    int16_t offset_accel_load; // Shift under load whilst accelerating
    int16_t offset_accel_idle; // Shift under no load whilst accelerating
    int16_t offset_decel_load; // Shift under load whilst decelerating
    int16_t offset_decel_idle; // Shift under no load whilst decelerating

    float bite_speed_accel_load; // Shift under load whilst accelerating
    float bite_speed_accel_idle; // Shift under no load whilst accelerating
    float bite_speed_decel_load; // Shift under load whilst decelerating
    float bite_speed_decel_idle; // Shift under no load whilst decelerating
};

// For now, we just do what EGS52 does
class AdaptationMap  {
public:
    AdaptationMap();
    // Reset map to everything default
    void reset();
    bool save();
    void perform_adaptation(SensorData* sensors, ProfileGearChange change, ShiftResponse response, bool upshift);
    int get_adaptation_offset(SensorData* sensors, ProfileGearChange change);
    float get_adaptation_speed(SensorData* sensors, ProfileGearChange change);
private:
    AdaptationData adapt_data[8];
    bool load_from_nvs();
};


#endif // __ADAPT_MAP_H__