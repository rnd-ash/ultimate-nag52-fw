#include "map_editor.h"
#include "kwp2000_defines.h"
#include "../nvs/eeprom_config.h"
#include "../maps.h"

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

uint8_t MapEditor::read_map_data(uint8_t map_id, uint16_t *dest_size, int16_t** buffer) {
    if (map_id >= 0x01 && map_id <= 0x0C) {
        // Shift map
        int16_t* b = (int16_t*)heap_caps_malloc(SHIFT_MAP_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM);
        if (b == nullptr) {
            ESP_LOGE("MAP_EDITOR_R", "Could not allocate read array!");
            return NRC_GENERAL_REJECT;
        }
        const char* name = map_id_to_name(map_id);
        if (name == nullptr) {
            ESP_LOGE("MAP_EDITOR_R", "map name is null!?");
            delete b;
            return NRC_GENERAL_REJECT;
        }
        if (EEPROM::read_nvs_map_data(map_id_to_name(map_id), b, nullptr, SHIFT_MAP_SIZE)) {
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
            return 0;
        } else {
            ESP_LOGE("MAP_EDITOR_W", "write_nvs_map_data failed!");
            return NRC_GENERAL_REJECT;
        }
    }  else {
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
}


