#ifndef _SHIFT_ADAPT_SYSTEM_H
#define _SHIFT_ADAPT_SYSTEM_H

#include <stdint.h>
#include "stored_map.h"
#include "common_structs.h"
#include "esp_err.h"

extern const char* CLUTCH_NAMES[5];

struct AdaptShiftRequest{
    uint16_t override_shift_torque;
} ;

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

struct AdaptPrefillData{
    int16_t pressure_offset_on_clutch;
    int16_t timing_offset;
    uint16_t torque_lim;
} ;

class ShiftAdaptationSystem  {
public:
    explicit ShiftAdaptationSystem(GearboxConfiguration* cfg_ptr);
    
    uint32_t check_prefill_adapt_conditions_start(SensorData* sensors, GearChange change);

    AdaptPrefillData get_prefill_adapt_data(GearChange change);

    esp_err_t reset(void);
    esp_err_t save(void);
    
    void debug_print_prefill_data();

private:
    bool set_prefill_cell_offset(StoredMap* dest, GearChange change, int16_t offset, int16_t pos_lim, int16_t neg_lim);
    GearboxConfiguration* gb_cfg;
    uint8_t pre_shift_pedal_pos = 0u;

    StoredMap* prefill_pressure_offset_map;
    StoredMap* prefill_time_offset_map;
    StoredMap* prefill_adapt_torque_limit_map;

    GearChange current_change;

    bool flared = false;
    uint64_t flare_time = 0;
};


#endif // ADAPT_MAP_H