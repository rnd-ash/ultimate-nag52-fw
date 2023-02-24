#ifndef _SHIFT_ADAPT_SYSTEM_H
#define _SHIFT_ADAPT_SYSTEM_H

#include <stdint.h>
#include "stored_map.h"
#include "common_structs.h"
#include "esp_err.h"

// Adaptation inputs:
// 1. The Gear change (8 different maps)
// 2. Transmission input speed (Which correlates to output speed) (0-6000RPM - 1000RPM increments)
// 3. Engine input torque (-20%-100% - 20% Load intervals) - 100% is 330Nm for small NAG, and 580Nm for large NAG
// 4. Desired shift speed (100-1000ms - 200ms intervals)
//
// Adaptation outputs:
// 1. MPC prefill padding adder
// 2. MPC prefill delay for release
// 3. SPC Prefill adder
// 4. SPC prefill time adder
// 5. SPC Target adder
//
// Adaptation limits (Same index listing as adaption outputs):
// 1. -500 - +500mBar
// 2. N/A
// 3. -500 - + 500mBar
// 4. -200 - +500mSec
// 5. -500 - +1500mBar

const int16_t LOAD_COL[7] = { -20, 0, 20, 40, 60, 80, 100 };
const int16_t RPM_COL[7] = {0, 1000, 2000, 3000, 4000, 5000, 6000};
const int16_t SHIFT_SPD_COL[5] = { 200, 400, 600, 800, 1000 };

typedef struct {
    int16_t mpc_prefill_adder;
    int16_t mpc_prefill_release_time_adder;
    int16_t spc_prefill_adder;
    int16_t spc_prefill_time_adder;
    int16_t spc_ramp_end_adder;
} AdaptationCell;

// Total cells = 245 * 8 = 1960.
// Cell size = 10 bytes (5x int16_t)
// Total memory for adaptation = ~20Kb

class ShiftAdaptationSystem  {
public:
    ShiftAdaptationSystem(void);
    esp_err_t reset(void);
    esp_err_t save(void);
private:
    StoredTcuMap* shift_adaptation_maps[8];
};


#endif // ADAPT_MAP_H