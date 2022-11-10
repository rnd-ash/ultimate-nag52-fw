#include "map_editor.h"
#include "kwp2000_defines.h"
#include "../nvs/eeprom_config.h"
#include "../maps.h"
#include "string.h"
#include "../speaker.h"
#include "../profiles.h"
#include "../pressure_manager.h"

StoredTcuMap* get_map(uint8_t map_id) {
    switch(map_id) {
        case A_UPSHIFT_MAP_ID:
            return agility->get_upshift_map();
        case C_UPSHIFT_MAP_ID:
            return comfort->get_upshift_map();
        case S_UPSHIFT_MAP_ID:
            return standard->get_upshift_map();
        case A_DOWNSHIFT_MAP_ID:
            return agility->get_downshift_map();
        case C_DOWNSHIFT_MAP_ID:
            return comfort->get_downshift_map();
        case S_DOWNSHIFT_MAP_ID:
            return standard->get_downshift_map();
        case WORKING_PRESSURE_MAP_ID:
            return pressure_manager->get_working_map();
        case PCS_CURRENT_MAP_ID:
            return pressure_manager->get_pcs_map();
        case TCC_PWM_MAP_ID:
            return pressure_manager->get_tcc_pwm_map();
        case FILL_TIME_MAP_ID:
            return pressure_manager->get_fill_time_map();
        case FILL_PRESSURE_MAP_ID:
            return pressure_manager->get_fill_pressure_map();
        default:
            return nullptr;
    }
}

#define CHECK_MAP(map_id) \
    StoredTcuMap* ptr = get_map(map_id); \
    if (ptr == nullptr) { \
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT; \
    }

uint8_t MapEditor::read_map_data(uint8_t map_id, bool is_default, uint16_t *dest_size_bytes, uint8_t** buffer) {
    CHECK_MAP(map_id)
    // Map valid
    int size = ptr->get_map_element_count();
    uint8_t* b = (uint8_t*)heap_caps_malloc((size*sizeof(int16_t)), MALLOC_CAP_SPIRAM);
    if (b == nullptr) {
        ESP_LOGE("MAP_EDITOR_R", "Could not allocate read array!");
        return NRC_GENERAL_REJECT;
    }
    if (is_default) {
        memcpy(b, ptr->get_default_map_data(), size*sizeof(int16_t));
    } else {
        memcpy(b, ptr->get_current_map_data(), size*sizeof(int16_t));
    }
    
    *buffer = b;
    *dest_size_bytes = size*sizeof(int16_t);
    return 0;
}

uint8_t MapEditor::read_map_metadata(uint8_t map_id, uint16_t *dest_size_bytes, uint8_t** buffer) {
    CHECK_MAP(map_id);
    // X meta, Y meta, KEY_NAME
    int16_t* x_ptr;
    int16_t* y_ptr;
    const char* k_ptr;
    uint16_t x_size;
    uint16_t y_size;
    uint16_t k_size;

    ptr->get_x_headers(&x_size, &x_ptr);
    ptr->get_y_headers(&y_size, &y_ptr);
    k_ptr = ptr->get_map_key_name();
    k_size = strlen(k_ptr);
    // 6 bytes for size data
    uint16_t size = 6+k_size+((x_size+y_size)*sizeof(int16_t));
    uint8_t* b = (uint8_t*)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (b == nullptr) {
        ESP_LOGE("MAP_EDITOR_M", "Could not allocate read array!");
        return NRC_GENERAL_REJECT;
    }
    b[1] = x_size >> 8;
    b[0] = x_size & 0xFF;
    b[3] = y_size >> 8;
    b[2] = y_size & 0xFF;
    b[5] = k_size >> 8;
    b[4] = k_size & 0xFF;
    memcpy(&b[6], x_ptr, x_size*sizeof(int16_t));
    memcpy(&b[6+(x_size*sizeof(int16_t))], y_ptr, y_size*sizeof(int16_t));
    memcpy(&b[6+((x_size+y_size)*sizeof(int16_t))], k_ptr, k_size);
    *buffer = b;
    *dest_size_bytes = size;
    return 0;
}
    
uint8_t MapEditor::write_map_data(uint8_t map_id, uint16_t dest_size, int16_t* buffer) {
    CHECK_MAP(map_id)
    if (ptr->replace_map_content(buffer, dest_size)) {
        return 0;
    } else {
        return NRC_GENERAL_REJECT;
    }
}

uint8_t MapEditor::burn_to_eeprom(uint8_t map_id) {
    CHECK_MAP(map_id)
    if (ptr->save_to_eeprom()) {
        return 0;
    } else {
        return NRC_GENERAL_REJECT;
    }
}

uint8_t MapEditor::reset_to_program_default(uint8_t map_id) {
    CHECK_MAP(map_id)
    if (ptr->reload_from_eeprom() && ptr->save_to_eeprom()) {
        return 0;
    } else {
        return NRC_GENERAL_REJECT;
    }
}

uint8_t MapEditor::undo_changes(uint8_t map_id) {
    CHECK_MAP(map_id)
    if (ptr->reload_from_eeprom()) {
        return 0;
    } else {
        return NRC_GENERAL_REJECT;
    }
}
