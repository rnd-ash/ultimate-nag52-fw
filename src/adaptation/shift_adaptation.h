#ifndef SHIFT_ADAPT_SYSTEM_H
#define SHIFT_ADAPT_SYSTEM_H

#include <stdint.h>
#include "stored_map.h"
#include "common_structs.h"
#include "esp_err.h"

extern const char* CLUTCH_NAMES[5];

/* unused */
// struct AdaptShiftRequest{
//     uint16_t override_shift_torque;
// } ;

// Flags for adapt cancelled 
enum AdaptCancelFlag : uint32_t {
    ADAPTABLE = 0,
    NOT_ADAPTABLE           = (uint32_t)1u << 0u,
    USER_CANCELLED          = (uint32_t)1u << 1u,
    ENGINE_NOT_ACK_TORQUE   = (uint32_t)1u << 2u,
    ESP_INTERVENTION        = (uint32_t)1u << 3u,
    PEDAL_DELTA_TOO_HIGH    = (uint32_t)1u << 4u,
    ATF_TEMP_TOO_HIGH       = (uint32_t)1u << 5u,
    ATF_TEMP_TOO_LOW        = (uint32_t)1u << 6u,
    INPUT_RPM_TOO_HIGH      = (uint32_t)1u << 7u,
    INPUT_RPM_TOO_LOW       = (uint32_t)1u << 8u,
    ENGINE_RPM_ERROR        = (uint32_t)1u << 9u,
    INPUT_RPM_ERROR         = (uint32_t)1u << 10u,
    ABS_RPM_ERROR           = (uint32_t)1u << 11u,
    INPUT_TRQ_TOO_HIGH      = (uint32_t)1u << 12u,
} ;

struct AdaptPrefillData{
    int16_t pressure_offset_on_clutch;
    int16_t timing_offset;
    uint16_t torque_lim;
} ;

class ShiftAdaptationSystem  {
public:
    explicit ShiftAdaptationSystem(GearboxConfiguration* cfg_ptr);
    
    uint32_t check_prefill_adapt_conditions_start(SensorData* sensors, ProfileGearChange change);

    // void record_shift_start(uint64_t time_into_shift, int overlap_start_ts, uint16_t mpc, uint16_t spc, ShiftClutchVelocity vel, uint16_t delta_rpm);
    //void record_shift_end(ShiftStage c_stage, uint64_t time_into_phase, uint16_t mpc, uint16_t spc);

    //void record_flare(ShiftStage when, uint64_t elapsed);
    // uint16_t get_overlap_end_shift_pressure(ProfileGearChange change, uint16_t selected_prefill_pressure);

    // AdaptPrefillData get_prefill_adapt_data(ProfileGearChange change);

    esp_err_t reset(void);
    esp_err_t save(void);
    
    // void debug_print_prefill_data(void);

private:
    // static bool set_prefill_cell_offset(StoredMap* dest, ProfileGearChange change, int16_t offset, int16_t pos_lim, int16_t neg_lim);
    GearboxConfiguration* gb_cfg;
    uint8_t pre_shift_pedal_pos = 0u;
    
    const int16_t prefill_x_headers[8] = {1,2,3,4,5,6,7};
    const int16_t prefill_y_headers[1] = {1};
    
    StoredMap* prefill_pressure_offset_map;
    StoredMap* prefill_time_offset_map;
    StoredMap* prefill_adapt_torque_limit_map;

    ProfileGearChange current_change;

    bool flared = false;
    uint64_t flare_time = 0;
};


#endif // ADAPT_MAP_H