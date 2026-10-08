#ifndef _SHIFT_ADAPT_SYSTEM_H
#define _SHIFT_ADAPT_SYSTEM_H

#include <cstdint>
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

    int16_t get_pulling_torque_offset(uint8_t shift_idx, uint16_t input_rpm, uint16_t load_percentage);
    int16_t get_pushing_torque_offset(uint8_t shift_idx, uint16_t input_rpm, uint16_t load_percentage);
    esp_err_t save(void);
    void offset_prefill_cycles(uint8_t shift_idx, int8_t offset);
    void offset_spc_pressure(uint8_t shift_idx, int16_t offset);

    void set_pulling_torque(uint8_t shift_idx, int16_t new_val, uint16_t input_rpm, uint16_t load_percentage);
    void set_pushing_torque(uint8_t shift_idx, int16_t new_val, uint16_t input_rpm, uint16_t load_percentage);
    StoredMap* get_pulling_torque_map(uint8_t shift_idx);
    StoredMap* get_pushing_torque_map(uint8_t shift_idx);
    esp_err_t reset();

    StoredMap* prefill_time_map;
    StoredMap* spc_offset_map;
    // X = RPM, Y = Load %
    StoredMap* pushing_trq_map[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    // X = RPM, Y = Load %
    StoredMap* pulling_trq_map[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};


private:
    bool init_ok = false;

};

extern ShiftAdaptationSystem* adaptation_manager;

#endif // ADAPT_MAP_H
