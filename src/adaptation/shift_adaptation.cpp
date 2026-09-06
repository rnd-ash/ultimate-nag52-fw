
#include "shift_adaptation.h"
#include <string.h>
#include <esp_log.h>
#include "nvs.h"
#include "nvs/eeprom_config.h"
#include "esp_check.h"
#include "nvs/module_settings.h"
#include "common_structs_ops.h"
#include "maps.h"
#include "nvs/all_keys.h"

// One cell per forward shift (see fwd_gearchange_egs_map_lookup_idx, 0-7)
#define ADAPT_MAP_SIZE (8u)

ShiftAdaptationSystem::ShiftAdaptationSystem()
{
    const int16_t adpt_map_x[ADAPT_MAP_SIZE] = {0,1,2,3,4,5,6,7};
    const int16_t adpt_map_y[1] = {1};
    // Drop any map that failed to load. Without this the pointer stays non-null
    // while its backing allocation is null, so the "nullptr != map" guards in
    // every accessor below pass and then dereference a null data array.
    // (Same pattern PressureManager/TorqueConverter/AbstractProfile use.)
    this->prefill_time_map = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_PREFILL_TIME, ADAPT_MAP_SIZE, adpt_map_x, adpt_map_y, ADAPT_MAP_SIZE, 1, GEAR_ADAPT_MAP);
    if (ESP_OK != this->prefill_time_map->init_status()) {
        delete this->prefill_time_map;
        this->prefill_time_map = nullptr;
    }
    this->applying_torque_offset = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_APPLYING_TRQ, ADAPT_MAP_SIZE, adpt_map_x, adpt_map_y, ADAPT_MAP_SIZE, 1, GEAR_ADAPT_MAP);
    if (ESP_OK != this->applying_torque_offset->init_status()) {
        delete this->applying_torque_offset;
        this->applying_torque_offset = nullptr;
    }
    this->freeing_torque_offset = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_FREEING_TRQ, ADAPT_MAP_SIZE, adpt_map_x, adpt_map_y, ADAPT_MAP_SIZE, 1, GEAR_ADAPT_MAP);
    if (ESP_OK != this->freeing_torque_offset->init_status()) {
        delete this->freeing_torque_offset;
        this->freeing_torque_offset = nullptr;
    }
    this->spc_offset_map = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_SPC_OFFSET, ADAPT_MAP_SIZE, adpt_map_x, adpt_map_y, ADAPT_MAP_SIZE, 1, GEAR_ADAPT_MAP);
    if (ESP_OK != this->spc_offset_map->init_status()) {
        delete this->spc_offset_map;
        this->spc_offset_map = nullptr;
    }
}

// Returns the writable cell for shift_idx, or nullptr if the map is unusable or
// the index is out of range. Every accessor goes through this so the bounds
// check cannot be forgotten at one call site.
static int16_t* adapt_cell(StoredMap* map, uint8_t shift_idx) {
    int16_t* ret = nullptr;
    if (nullptr != map && shift_idx < ADAPT_MAP_SIZE) {
        int16_t* data = map->get_current_data();
        if (nullptr != data) {
            ret = &data[shift_idx];
        }
    }
    return ret;
}

esp_err_t ShiftAdaptationSystem::save(void) {
    if (nullptr != this->prefill_time_map) {
        this->prefill_time_map->save_to_eeprom();
    }
    if (nullptr != this->applying_torque_offset) {
        this->applying_torque_offset->save_to_eeprom();
    }
    if (nullptr != this->freeing_torque_offset) {
        this->freeing_torque_offset->save_to_eeprom();
    }
    if (nullptr != this->spc_offset_map) {
        this->spc_offset_map->save_to_eeprom();
    }
    return ESP_OK;
}

int8_t ShiftAdaptationSystem::get_prefill_cycles_offset(uint8_t shift_idx) {
    const int16_t* cell = adapt_cell(this->prefill_time_map, shift_idx);
    return (nullptr != cell) ? (int8_t)*cell : 0;
}

int16_t ShiftAdaptationSystem::get_adapt_spc_offset(uint8_t shift_idx) {
    const int16_t* cell = adapt_cell(this->spc_offset_map, shift_idx);
    return (nullptr != cell) ? *cell : 0;
}

int16_t ShiftAdaptationSystem::get_freeing_torque_offset(uint8_t shift_idx) {
    const int16_t* cell = adapt_cell(this->freeing_torque_offset, shift_idx);
    return (nullptr != cell) ? *cell : 0;
}

int16_t ShiftAdaptationSystem::get_applying_torque_offset(uint8_t shift_idx) {
    const int16_t* cell = adapt_cell(this->applying_torque_offset, shift_idx);
    return (nullptr != cell) ? *cell : 0;
}

void ShiftAdaptationSystem::offset_prefill_cycles(uint8_t shift_idx, int8_t offset) {
    int16_t* cell = adapt_cell(this->prefill_time_map, shift_idx);
    if (nullptr != cell) {
        *cell += offset;
        if (*cell > ADP_CURRENT_SETTINGS.prefill_max_time_delta) {
            *cell = ADP_CURRENT_SETTINGS.prefill_max_time_delta;
            ESP_LOGW("ADAPT", "Prefill cycles max limit reached");
        } else if (*cell < -ADP_CURRENT_SETTINGS.prefill_max_time_delta) {
            *cell = -ADP_CURRENT_SETTINGS.prefill_max_time_delta;
            ESP_LOGW("ADAPT", "Prefill cycles min limit reached");
        } else {
            ESP_LOGI("ADAPT", "Prefill cycles offset by %d to %d", offset, *cell);
        }
    }
}

void ShiftAdaptationSystem::offset_spc_pressure(uint8_t shift_idx, int16_t offset) {
    int16_t* cell = adapt_cell(this->spc_offset_map, shift_idx);
    if (nullptr != cell) {
        *cell += offset;
        ESP_LOGI("ADAPT", "SPC pressure offset by %d to %d", offset, *cell);
    }
}

void ShiftAdaptationSystem::offset_freeing_trq(uint8_t shift_idx, int16_t offset) {
    int16_t* cell = adapt_cell(this->freeing_torque_offset, shift_idx);
    if (nullptr != cell) {
        *cell += offset;
        ESP_LOGI("ADAPT", "Free. Trq offset by %d to %d", offset, *cell);
    }
}

void ShiftAdaptationSystem::offset_applying_trq(uint8_t shift_idx, int16_t offset) {
    int16_t* cell = adapt_cell(this->applying_torque_offset, shift_idx);
    if (nullptr != cell) {
        *cell += offset;
        ESP_LOGI("ADAPT", "Appl. Trq offset by %d to %d", offset, *cell);
    }
}

esp_err_t ShiftAdaptationSystem::reset() {
    if (nullptr != this->prefill_time_map) {
        this->prefill_time_map->reset_from_flash();
    }
    if (nullptr != this->spc_offset_map) {
        this->spc_offset_map->reset_from_flash();
    }
    if (nullptr != this->freeing_torque_offset) {
        this->freeing_torque_offset->reset_from_flash();
    }
    if (nullptr != this->applying_torque_offset) {
        this->applying_torque_offset->reset_from_flash();
    }
    return ESP_OK;
}

ShiftAdaptationSystem* adaptation_manager = nullptr;