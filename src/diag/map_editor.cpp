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
            return pressure_manager->get_fill_time_map();
        default:
            return nullptr;
    }
}

#define CHECK_MAP(map_id) \
    StoredTcuMap* ptr = get_map(map_id); \
    if (ptr == nullptr) { \
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT; \
    }

uint8_t MapEditor::read_map_data(uint8_t map_id, bool is_default, uint16_t *dest_size, int16_t** buffer) {
    CHECK_MAP(map_id)
    // Map valid
    int size = ptr->get_map_element_count();
    int16_t* b = (int16_t*)heap_caps_malloc(size*sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (b == nullptr) {
        ESP_LOGE("MAP_EDITOR_R", "Could not allocate read array!");
        return NRC_GENERAL_REJECT;
    }
    const int16_t* data;
    if (is_default) {
        data = ptr->get_default_map_data();
    } else {
        data = (const int16_t*)ptr->get_current_map_data();
    }
    memcpy(b, data, size*sizeof(int16_t));
    *buffer = b;
    *dest_size = size*sizeof(int16_t);
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
