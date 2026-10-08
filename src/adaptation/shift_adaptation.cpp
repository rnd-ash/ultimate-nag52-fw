
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
#include "stored_map.h"

const int16_t adpt_trq_x[4] = {1000, 1500, 2000, 4000};
const int16_t adpt_trq_y_pull[5] = {0, 20, 50, 100, 150};
const int16_t adpt_trq_y_push[4] = {0, 5, 10, 20};

StoredMap* make_trq_pull_map(const char* key) {
    StoredMap* map = new StoredMap(key, GEAR_TRQ_PULL_ADAPT_MAP_SIZE, adpt_trq_x, adpt_trq_y_pull, 4, 5, GEAR_PULLING_TRQ_ADAPT_MAP);
    if (ESP_OK == map->init_status()) {
        return map;
    } else {
        delete map;
        ESP_LOGE("MapInit", "%s failed to allocate", key);
        return nullptr;
    }
}

StoredMap* make_trq_push_map(const char* key) {
    StoredMap* map = new StoredMap(key, GEAR_TRQ_PUSH_ADAPT_MAP_SIZE, adpt_trq_x, adpt_trq_y_push, 4, 4, GEAR_PUSHING_TRQ_ADAPT_MAP);
    if (ESP_OK == map->init_status()) {
        return map;
    } else {
        delete map;
        ESP_LOGE("MapInit", "%s failed to allocate", key);
        return nullptr;
    }
}

ShiftAdaptationSystem::ShiftAdaptationSystem()
{
    const int16_t adpt_map_x[8] = {0,1,2,3,4,5,6,7};
    const int16_t adpt_map_y[1] = {1};
    // Adapt map allocation is non fatal here, since we can substitute 0 values
    this->prefill_time_map = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_PREFILL_TIME, 8*1, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    this->spc_offset_map = new StoredMap(NVS_KEY_MAP_NAME_ADAPT_SPC_OFFSET, 8*1, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    
    this->pushing_trq_map[0] = make_trq_push_map(NVS_KEY_PUSHING_TRQ_ADP_D12);
    this->pushing_trq_map[1] = make_trq_push_map(NVS_KEY_PUSHING_TRQ_ADP_D23);
    this->pushing_trq_map[2] = make_trq_push_map(NVS_KEY_PUSHING_TRQ_ADP_D34);
    this->pushing_trq_map[3] = make_trq_push_map(NVS_KEY_PUSHING_TRQ_ADP_D45);
    this->pushing_trq_map[4] = make_trq_push_map(NVS_KEY_PUSHING_TRQ_ADP_D21);
    this->pushing_trq_map[5] = make_trq_push_map(NVS_KEY_PUSHING_TRQ_ADP_D32);
    this->pushing_trq_map[6] = make_trq_push_map(NVS_KEY_PUSHING_TRQ_ADP_D43);
    this->pushing_trq_map[7] = make_trq_push_map(NVS_KEY_PUSHING_TRQ_ADP_D54);

    this->pulling_trq_map[0] = make_trq_pull_map(NVS_KEY_PULLING_TRQ_ADP_D12);
    this->pulling_trq_map[1] = make_trq_pull_map(NVS_KEY_PULLING_TRQ_ADP_D23);
    this->pulling_trq_map[2] = make_trq_pull_map(NVS_KEY_PULLING_TRQ_ADP_D34);
    this->pulling_trq_map[3] = make_trq_pull_map(NVS_KEY_PULLING_TRQ_ADP_D45);
    this->pulling_trq_map[4] = make_trq_pull_map(NVS_KEY_PULLING_TRQ_ADP_D21);
    this->pulling_trq_map[5] = make_trq_pull_map(NVS_KEY_PULLING_TRQ_ADP_D32);
    this->pulling_trq_map[6] = make_trq_pull_map(NVS_KEY_PULLING_TRQ_ADP_D43);
    this->pulling_trq_map[7] = make_trq_pull_map(NVS_KEY_PULLING_TRQ_ADP_D54);

}

esp_err_t ShiftAdaptationSystem::save(void) {
    if (nullptr != this->prefill_time_map) {
        this->prefill_time_map->save_to_eeprom();
    }
    if (nullptr != this->spc_offset_map) {
        this->spc_offset_map->save_to_eeprom();
    }
    for (int map_idx = 0; map_idx < 8; map_idx++) {
        StoredMap* pulling = this->get_pulling_torque_map(map_idx);
        StoredMap* pushing = this->get_pushing_torque_map(map_idx);
        if (nullptr != pulling) {
            pulling->save_to_eeprom();
        }
        if (nullptr != pushing) {
            pushing->save_to_eeprom();
        }
    }
    return ESP_OK;
}

StoredMap* ShiftAdaptationSystem::get_pulling_torque_map(uint8_t shift_idx) {
    if (shift_idx < 8) {
        return this->pulling_trq_map[shift_idx];
    } else {
        return nullptr;
    }
}

StoredMap* ShiftAdaptationSystem::get_pushing_torque_map(uint8_t shift_idx) {
    if (shift_idx < 8) {
        return this->pushing_trq_map[shift_idx];
    } else {
        return nullptr;
    }
}

int8_t ShiftAdaptationSystem::get_prefill_cycles_offset(uint8_t shift_idx) {
    int16_t ret = 0;
    if (nullptr != this->prefill_time_map) {
        ret = this->prefill_time_map->get_current_data()[shift_idx];
    }
    return ret;
}

int16_t ShiftAdaptationSystem::get_adapt_spc_offset(uint8_t shift_idx) {
    int16_t ret = 0;
    if (nullptr != this->spc_offset_map) {
        ret = this->spc_offset_map->get_current_data()[shift_idx];
    }
    return ret;
}

int16_t ShiftAdaptationSystem::get_pulling_torque_offset(uint8_t shift_idx, uint16_t input_rpm, uint16_t load_percentage) {
    int16_t ret = 0;
    StoredMap* map = this->get_pulling_torque_map(shift_idx);
    if (nullptr != map) {
        ret = map->get_value(input_rpm, load_percentage);
    }
    return ret;
}

int16_t ShiftAdaptationSystem::get_pushing_torque_offset(uint8_t shift_idx, uint16_t input_rpm, uint16_t load_percentage) {
    int16_t ret = 0;
    StoredMap* map = this->get_pushing_torque_map(shift_idx);
    if (nullptr != map) {
        ret = map->get_value(input_rpm, load_percentage);
    }
    return ret;
}

void ShiftAdaptationSystem::offset_prefill_cycles(uint8_t shift_idx, int8_t offset) {
    if (nullptr != this->prefill_time_map) {
        int16_t* ptr = this->prefill_time_map->get_current_data();
        ptr[shift_idx] += offset;
        if (ptr[shift_idx] > ADP_CURRENT_SETTINGS.prefill_max_time_delta) {
            ptr[shift_idx] = ADP_CURRENT_SETTINGS.prefill_max_time_delta;
            ESP_LOGW("ADAPT", "Prefill cycles max limit reached");
        } else if (ptr[shift_idx] < -ADP_CURRENT_SETTINGS.prefill_max_time_delta) {
            ptr[shift_idx] = -ADP_CURRENT_SETTINGS.prefill_max_time_delta;
            ESP_LOGW("ADAPT", "Prefill cycles min limit reached");
        } else {
            ESP_LOGI("ADAPT", "Prefill cycles offset by %d to %d", offset, ptr[shift_idx]);
        }
    }
}

void ShiftAdaptationSystem::offset_spc_pressure(uint8_t shift_idx, int16_t offset) {
    if (nullptr != this->spc_offset_map) {
        int16_t* ptr = this->spc_offset_map->get_current_data();
        ptr[shift_idx] += offset;
        ESP_LOGI("ADAPT", "SPC pressure offset by %d to %d", offset, ptr[shift_idx]);
    }
}

void ShiftAdaptationSystem::set_pulling_torque(uint8_t shift_idx, int16_t new_val, uint16_t input_rpm, uint16_t load_percentage) {
    StoredMap* map = this->get_pulling_torque_map(shift_idx);
    if (nullptr != map) {
        map->add_value(new_val, input_rpm, load_percentage, 10.0);
        ESP_LOGI("ADAPT", "Pulling Trq Adjusted to %d Nm for [%d RPM][%d %]", new_val, input_rpm, load_percentage);
    } else {
        ESP_LOGW("ADAPT", "Pulling torque could not be adjusted (Null map)");
    }
}

void ShiftAdaptationSystem::set_pushing_torque(uint8_t shift_idx, int16_t new_val, uint16_t input_rpm, uint16_t load_percentage) {
    StoredMap* map = this->get_pushing_torque_map(shift_idx);
    if (nullptr != map) {
        map->add_value(new_val, input_rpm, load_percentage, 10.0);
        ESP_LOGI("ADAPT", "Pushing Trq Adjusted to %d Nm for [%d RPM][%d %]", new_val, input_rpm, load_percentage);
    } else {
        ESP_LOGW("ADAPT", "Pushing torque could not be adjusted (Null map)");
    }
}

esp_err_t ShiftAdaptationSystem::reset() {
    
    if (nullptr != this->prefill_time_map) {
        this->prefill_time_map->reset_from_flash();
    }
    if (nullptr != this->spc_offset_map) {
        this->spc_offset_map->reset_from_flash();
    }
    for (int map_idx = 0; map_idx < 8; map_idx++) {
        StoredMap* pulling = this->get_pulling_torque_map(map_idx);
        StoredMap* pushing = this->get_pushing_torque_map(map_idx);
        if (nullptr != pulling) {
            pulling->reset_from_flash();
        }
        if (nullptr != pushing) {
            pushing->reset_from_flash();
        }
    }
    return ESP_OK;
}

ShiftAdaptationSystem* adaptation_manager = nullptr;
