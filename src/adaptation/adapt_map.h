#ifndef ADAPT_MAP_H
#define ADAPT_MAP_H

#include <stdint.h>
#include <gearbox_config.h>
#include "common_structs.h"

struct AdaptationCell {
    int16_t spc_fill_adder;
    int16_t mpc_fill_adder;
    int16_t fill_time_adder;
};

struct AdaptationData {
    AdaptationCell trq_neg;
    AdaptationCell trq_25;
    AdaptationCell trq_50;
    AdaptationCell trq_75;
};

const static AdaptationCell DEFAULT_CELL {
    .spc_fill_adder = 0,
    .mpc_fill_adder = 0,
    .fill_time_adder = 0
};

// For now, we just do what EGS52 does
class AdaptationMap  {
public:
    AdaptationMap(void);
    // Reset map to everything default
    void reset(void);
    bool save(void);
    void perform_adaptation(SensorData* sensors, ShiftReport* rpt, ProfileGearChange change, bool is_valid_rpt, uint16_t gb_max_torque);
    AdaptationCell* get_adapt_cell(SensorData* sensors, ProfileGearChange change, uint16_t gb_max_torque);
private:
    const uint16_t ADAPT_RPM_LIMIT = 2500u;
    const int16_t ADAPT_TEMP_THRESH = 60;
    const int16_t ADAPT_TEMP_LIMIT = 120;
    inline static AdaptationCell* get_adapt_cell_from_torque(SensorData *sensors, uint16_t gb_max_torque, uint16_t adaptation_idx, AdaptationMap* adaptationmap_var);
    inline static uint16_t get_idx_from_change(ProfileGearChange change);
    AdaptationData adapt_data[8];
    bool load_from_nvs(void);
};


#endif // ADAPT_MAP_H