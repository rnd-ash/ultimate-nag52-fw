#include "map_editor.h"
#include "kwp2000_defines.h"
#include "../nvs/eeprom_config.h"
#include "../maps.h"
#include "string.h"
#include "../speaker.h"
#include "../profiles.h"

const char* map_id_to_name(uint8_t map_id) {
    switch(map_id) {
        case MAP_ID_S_DIESEL_UPSHIFT:
            return MAP_NAME_S_DIESEL_UPSHIFT;
        case MAP_ID_S_DIESEL_DOWNSHIFT:
            return MAP_NAME_S_DIESEL_DOWNSHIFT;
        case MAP_ID_S_PETROL_UPSHIFT:
            return MAP_NAME_S_PETROL_UPSHIFT;
        case MAP_ID_S_PETROL_DOWNSHIFT:
            return MAP_NAME_S_PETROL_DOWNSHIFT;
        case MAP_ID_C_DIESEL_UPSHIFT:
            return MAP_NAME_C_DIESEL_UPSHIFT;
        case MAP_ID_C_DIESEL_DOWNSHIFT:
            return MAP_NAME_C_DIESEL_DOWNSHIFT;
        case MAP_ID_C_PETROL_UPSHIFT:
            return MAP_NAME_C_PETROL_UPSHIFT;
        case MAP_ID_C_PETROL_DOWNSHIFT:
            return MAP_NAME_C_PETROL_DOWNSHIFT;
        case MAP_ID_A_DIESEL_UPSHIFT:
            return MAP_NAME_A_DIESEL_UPSHIFT;
        case MAP_ID_A_DIESEL_DOWNSHIFT:
            return MAP_NAME_A_DIESEL_DOWNSHIFT;
        case MAP_ID_A_PETROL_UPSHIFT:
            return MAP_NAME_A_PETROL_UPSHIFT;
        case MAP_ID_A_PETROL_DOWNSHIFT:
            return MAP_NAME_A_PETROL_DOWNSHIFT; 
        default:
            return nullptr;
    }
}

const int16_t* get_default_map(uint8_t map_id) {
    switch(map_id) {
        case MAP_ID_S_DIESEL_UPSHIFT:
            return S_DIESEL_UPSHIFT_MAP;
        case MAP_ID_S_DIESEL_DOWNSHIFT:
            return S_DIESEL_DOWNSHIFT_MAP;
        case MAP_ID_S_PETROL_UPSHIFT:
            return S_PETROL_UPSHIFT_MAP;
        case MAP_ID_S_PETROL_DOWNSHIFT:
            return S_PETROL_DOWNSHIFT_MAP;
        case MAP_ID_C_DIESEL_UPSHIFT:
            return C_DIESEL_UPSHIFT_MAP;
        case MAP_ID_C_DIESEL_DOWNSHIFT:
            return C_DIESEL_DOWNSHIFT_MAP;
        case MAP_ID_C_PETROL_UPSHIFT:
            return C_PETROL_UPSHIFT_MAP;
        case MAP_ID_C_PETROL_DOWNSHIFT:
            return C_PETROL_DOWNSHIFT_MAP;
        case MAP_ID_A_DIESEL_UPSHIFT:
            return A_DIESEL_UPSHIFT_MAP;
        case MAP_ID_A_DIESEL_DOWNSHIFT:
            return A_DIESEL_DOWNSHIFT_MAP;
        case MAP_ID_A_PETROL_UPSHIFT:
            return A_PETROL_UPSHIFT_MAP;
        case MAP_ID_A_PETROL_DOWNSHIFT:
            return A_PETROL_DOWNSHIFT_MAP;
        default:
            return nullptr;
    }
}

uint8_t MapEditor::read_map_data(uint8_t map_id, uint16_t *dest_size, int16_t** buffer) {
    uint8_t map_idx = map_id & 0b1111111;
    bool is_default_map = (map_id & 0b10000000) != 0;
    if (map_idx >= 0x01 && map_idx <= 0x0C) {
        // Shift map
        int16_t* b = (int16_t*)heap_caps_malloc(SHIFT_MAP_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM);
        if (b == nullptr) {
            ESP_LOGE("MAP_EDITOR_R", "Could not allocate read array!");
            return NRC_GENERAL_REJECT;
        }
        if (is_default_map) {
            // Read default map from flash
            const int16_t* default_map = get_default_map(map_idx);
            if (default_map == nullptr) {
                ESP_LOGE("MAP_EDITOR_R", "default map is invalid!?");
                delete b;
                return NRC_GENERAL_REJECT;
            }
            memcpy(b, default_map, SHIFT_MAP_SIZE*sizeof(int16_t));
            *buffer = b;
            *dest_size = SHIFT_MAP_SIZE*sizeof(int16_t);
            return 0;
        } else {
            const char* name = map_id_to_name(map_idx);
            if (name == nullptr) {
                ESP_LOGE("MAP_EDITOR_R", "map name is null!?");
                delete b;
                return NRC_GENERAL_REJECT;
            }
            if (EEPROM::read_nvs_map_data(name, b, nullptr, SHIFT_MAP_SIZE)) {
                // OK, copy the pointer
                *buffer = b;
                *dest_size = SHIFT_MAP_SIZE*sizeof(int16_t);
                return 0;
            } else {
                // De-allocate
                ESP_LOGE("MAP_EDITOR_R", "read_nvs_map_data failed!");
                delete b;
                return NRC_GENERAL_REJECT;
            }
        }
    } else {
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
}
    
uint8_t MapEditor::write_map_data(uint8_t map_id, uint16_t dest_size, int16_t* buffer) {
    if (map_id >= 0x01 && map_id <= 0x0C) {
        const char* name = map_id_to_name(map_id);
        if (name == nullptr) {
            ESP_LOGE("MAP_EDITOR_W", "map name is null!?");
            return NRC_GENERAL_REJECT;
        }
        if (dest_size != SHIFT_MAP_SIZE*sizeof(int16_t)) {
            ESP_LOGE("MAP_EDITOR_W", "Buffer has %d bytes. Shift map needs %d bytes", dest_size, sizeof(int16_t)*SHIFT_MAP_SIZE);
            return NRC_GENERAL_REJECT;
        }
        if (EEPROM::write_nvs_map_data(name, buffer, SHIFT_MAP_SIZE)) {
            spkr.send_note(1500, 300, 310);

            return 0;
        } else {
            ESP_LOGE("MAP_EDITOR_W", "write_nvs_map_data failed!");
            return NRC_GENERAL_REJECT;
        }
    }  else {
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
}

uint8_t MapEditor::trigger_reload(uint8_t prof_id) {
    uint8_t ret = 0;
    if (prof_id == standard->get_profile_id()) {
        standard->reload_data();
    } else if (prof_id == comfort->get_profile_id()) {
        comfort->reload_data();
    } else if (prof_id == agility->get_profile_id()) {
        agility->reload_data();
    } else {
        ret = NRC_REQUEST_OUT_OF_RANGE;
    }
    return ret;
}
