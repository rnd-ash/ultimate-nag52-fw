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
#define ADAPT_TEMP_LIMIT 120

struct AdaptationCell {
    int16_t fill_pressure_mbar;
    int16_t fill_time_adder;
};

struct AdaptationData {
    AdaptationCell accel_load; // Shift under load whilst accelerating
    AdaptationCell accel_idle; // Shift under no load whilst accelerating
    AdaptationCell decel_load; // Shift under load whilst decelerating
    AdaptationCell decel_idle; // Shift under no load whilst decelerating
};

const static AdaptationCell DEFAULT_CELL {
    .fill_pressure_mbar = 0,
    .fill_time_adder = 0
};

// For now, we just do what EGS52 does
class AdaptationMap  {
public:
    AdaptationMap();
    // Reset map to everything default
    void reset();
    bool save();
    void perform_adaptation(SensorData* sensors, ShiftReport* rpt, ProfileGearChange change, bool is_valid_rpt, uint16_t gb_max_torque);
    const AdaptationCell* get_adapt_cell(SensorData* sensors, ProfileGearChange change);
private:
    AdaptationData adapt_data[8];
    bool load_from_nvs();
};


#endif // __ADAPT_MAP_H__