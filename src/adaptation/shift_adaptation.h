#ifndef _SHIFT_ADAPT_SYSTEM_H
#define _SHIFT_ADAPT_SYSTEM_H

#include <stdint.h>
#include "stored_map.h"
#include "common_structs.h"
#include "esp_err.h"

extern const char* CLUTCH_NAMES[5];

typedef struct {
    uint16_t override_shift_torque;
} AdaptShiftRequest;

// Flags for adapt cancelled 
typedef enum {
    ADAPTABLE = 0,
    NOT_ADAPTABLE = 1 << 0,
    USER_CANCELLED = 1 << 1,
    ENGINE_NOT_ACK_TORQUE = 1 << 2,
    ESP_INTERVENTION = 1 << 3,
    PEDAL_DELTA_TOO_HIGH = 1 << 4,
    ATF_TEMP_TOO_HIGH = 1 << 5,
    ATF_TEMP_TOO_LOW = 1 << 6,
    INPUT_RPM_TOO_HIGH = 1 << 7,
    INPUT_RPM_TOO_LOW = 1 << 8,
    ENGINE_RPM_ERROR = 1 << 9,
    INPUT_RPM_ERROR = 1 << 10,
    ABS_RPM_ERROR = 1 << 11,
    INPUT_TRQ_TOO_HIGH = 1 << 12,
} AdaptCancelFlag;

typedef struct {
    uint16_t start_mpc;
    uint16_t end_mpc;
} AdaptOverlapData;

typedef struct {
    int16_t pressure_offset;
    int16_t timing_offset;
} AdaptPrefillData;

class ShiftAdaptationSystem  {
public:
    ShiftAdaptationSystem(GearboxConfiguration* cfg_ptr);
    
    uint32_t check_prefill_adapt_conditions_start(SensorData* sensors, ProfileGearChange change);

    void record_shift_start(ShiftStage c_stage, uint64_t time_into_phase, uint16_t mpc, uint16_t spc, ShiftClutchVelocity vel, uint16_t target_min_fill_done, uint16_t target_max_fill_done);
    void record_shift_end(ShiftStage c_stage, uint64_t time_into_phase, uint16_t mpc, uint16_t spc);

    void record_flare(ShiftStage when, uint64_t elapsed);
    AdaptOverlapData get_overlap_data();

    AdaptPrefillData get_prefill_adapt_data(Clutch to_apply);

    esp_err_t reset(void);
    esp_err_t save(void);

    void debug_print_prefill_data();

private:
    bool set_prefill_cell_offset(StoredMap* dest, Clutch clutch, int16_t offset, int16_t pos_lim, int16_t neg_lim);
    GearboxConfiguration* gb_cfg;
    uint8_t pre_shift_pedal_pos;

    StoredMap* prefill_pressure_offset_map;
    StoredMap* prefill_time_offset_map;

    Clutch to_apply;

    bool flared = false;
    ShiftStage flare_location = ShiftStage::Bleed;
    uint64_t flare_time = 0;
};


#endif // ADAPT_MAP_H