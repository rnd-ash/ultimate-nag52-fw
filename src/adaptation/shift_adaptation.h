#ifndef _SHIFT_ADAPT_SYSTEM_H
#define _SHIFT_ADAPT_SYSTEM_H

#include <stdint.h>
#include "stored_map.h"
#include "common_structs.h"
#include "esp_err.h"

class ShiftAdaptationSystem  {
public:
    ShiftAdaptationSystem();
    void init_shift();
    void update();
    int8_t get_prefill_cycles_offset(uint8_t shift_idx);
    int16_t get_adapt_spc_offset(uint8_t shift_idx);
    int16_t get_freeing_torque_offset(uint8_t shift_idx);
    int16_t get_applying_torque_offset(uint8_t shift_idx);
    esp_err_t save(void);
    void offset_prefill_cycles(uint8_t shift_idx, int8_t offset);
    void offset_spc_pressure(uint8_t shift_idx, int8_t offset);

    void offset_freeing_trq(uint8_t shift_idx, int16_t offset);
    void offset_applying_trq(uint8_t shift_idx, int16_t offset);
    esp_err_t reset();
private:
    bool init_ok = false;
    StoredMap* prefill_time_map;

    StoredMap* applying_torque_offset;
    StoredMap* freeing_torque_offset;

    StoredMap* spc_offset_map;
};

#endif // ADAPT_MAP_H