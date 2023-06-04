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

class ShiftAdaptationSystem  {
public:
    ShiftAdaptationSystem(void);
    bool check_prefill_adapt_request(SensorData* sensors, ProfileGearChange change);
    void get_adapted_prefeill_data(ProfileGearChange change);
    esp_err_t reset(void);
    esp_err_t save(void);
private:
};


#endif // ADAPT_MAP_H