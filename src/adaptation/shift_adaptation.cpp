
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

ShiftAdaptationSystem::ShiftAdaptationSystem()
{
    const int16_t adpt_map_x[8] = {0,1,2,3,4,5,6,7};
    const int16_t adpt_map_y[1] = {1};
    this->prefill_time_map = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_PREFILL_TIME, 8*1, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    this->applying_torque_offset = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_APPLYING_TRQ, 8*1, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    this->freeing_torque_offset = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_FREEING_TRQ, 8*1, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    this->spc_offset_map = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_SPC_OFFSET, 8*1, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
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
    int16_t ret = 0;
    if (nullptr != this->prefill_time_map) {
        ret = this->prefill_time_map->get_current_data()[shift_idx];
    }
    return ret;
}

int16_t ShiftAdaptationSystem::get_freeing_torque_offset(uint8_t shift_idx) {
    int16_t ret = 0;
    if (nullptr != this->freeing_torque_offset) {
        ret = this->freeing_torque_offset->get_current_data()[shift_idx];
    }
    return ret;
}

int16_t ShiftAdaptationSystem::get_applying_torque_offset(uint8_t shift_idx) {
    int16_t ret = 0;
    if (nullptr != this->applying_torque_offset) {
        ret = this->applying_torque_offset->get_current_data()[shift_idx];
    }
    return ret;
}

void ShiftAdaptationSystem::offset_prefill_cycles(uint8_t shift_idx, int8_t offset) {
    if (nullptr != this->prefill_time_map) {
        int16_t* ptr = this->prefill_time_map->get_current_data();
        ptr[shift_idx] += offset;
        if (ptr[shift_idx] > ADP_CURRENT_SETTINGS.prefill_max_time_delta) {
            ptr[shift_idx] = ADP_CURRENT_SETTINGS.prefill_max_time_delta;
            ESP_LOGW("ADAPT", "Prefill cycles min limit reached");
        } else if (ptr[shift_idx] < -ADP_CURRENT_SETTINGS.prefill_max_time_delta) {
            ptr[shift_idx] = -ADP_CURRENT_SETTINGS.prefill_max_time_delta;
            ESP_LOGW("ADAPT", "Prefill cycles min limit reached");
        } else {
            ESP_LOGI("ADAPT", "Prefill cycles offset by %d to %d", offset, ptr[shift_idx]);
        }
    }
}

void ShiftAdaptationSystem::offset_freeing_trq(uint8_t shift_idx, int16_t offset) {
    if (nullptr != this->freeing_torque_offset) {
        int16_t* ptr = this->freeing_torque_offset->get_current_data();
        ptr[shift_idx] += offset;
        ESP_LOGI("ADAPT", "Free. Trq offset by %d to %d", offset, ptr[shift_idx]);
    }
}

void ShiftAdaptationSystem::offset_applying_trq(uint8_t shift_idx, int16_t offset) {
    if (nullptr != this->applying_torque_offset) {
        int16_t* ptr = this->applying_torque_offset->get_current_data();
        ptr[shift_idx] += offset;
        ESP_LOGI("ADAPT", "Appl. Trq offset by %d to %d", offset, ptr[shift_idx]);
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