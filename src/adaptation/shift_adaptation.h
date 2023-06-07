#ifndef _SHIFT_ADAPT_SYSTEM_H
#define _SHIFT_ADAPT_SYSTEM_H

#include <stdint.h>
#include "stored_map.h"
#include "common_structs.h"
#include "esp_err.h"

// Adaptation inputs:
// 1. Clutch to apply (To prefill)

// 0%, 10%, 25%, 50%

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

#define CELL_ID_NEG_TRQ 0
#define CELL_ID_10_PST_TRQ 1
#define CELL_ID_25_PST_TRQ 2

class ShiftAdaptationSystem  {
public:
    ShiftAdaptationSystem(GearboxConfiguration* cfg_ptr);
    
    uint32_t check_prefill_adapt_conditions_start(SensorData* sensors, ProfileGearChange change, int16_t* dest_trq_limit);
    uint32_t check_prefill_adapt_conditions_shift(SensorData* sensors, EgsBaseCan* can);

    uint32_t prefill_adapt_step(int shift_progress, SensorData* sensors);

    void get_adapted_prefeill_data(ProfileGearChange change);

    bool prefill_adapt_step();
    void on_overlap_start(uint64_t timestamp, uint64_t expected_shift_time, int shift_progress_percent);
    void do_prefill_overlap_check(uint64_t timestamp, bool flaring, int shift_progress_percent);

    void notify_early_shift(Clutch to_apply);

    int16_t get_prefill_pressure_offset(int16_t trq_nm, Clutch to_apply);

    esp_err_t reset(void);
    esp_err_t save(void);

    void debug_print_offset_array();

private:
    bool offset_cell_value(StoredMap* map, Clutch clutch, uint8_t load_cell_idx, int16_t offset);
    uint8_t cell_idx_prefill;
    int16_t requested_trq;
    GearboxConfiguration* gb_cfg;
    uint8_t pre_shift_pedal_pos;
    StoredMap* prefill_pressure_offset_map;
    StoredMap* prefill_time_offset_map;
    uint64_t last_overlap_check = 0;
    uint64_t expected_shift_time = 0;
    uint64_t overlap_start_time = 0;
    Clutch to_apply;
    bool flare_notified = false;
    int last_shift_progress = 0;
};


#endif // ADAPT_MAP_H