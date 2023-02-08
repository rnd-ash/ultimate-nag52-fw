#include "map_editor.h"
#include "kwp2000_defines.h"
#include "nvs/eeprom_config.h"
#include "../maps.h"
#include "string.h"
#include "speaker.h"
#include "profiles.h"
#include "pressure_manager.h"
#include "gearbox.h"

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
        case A_UPTIME_MAP_ID:
            return agility->get_upshift_time_map();
        case A_DNTIME_MAP_ID:
            return agility->get_downshift_time_map();
        case S_UPTIME_MAP_ID:
            return standard->get_upshift_time_map();
        case S_DNTIME_MAP_ID:
            return standard->get_downshift_time_map();
        case C_UPTIME_MAP_ID:
            return comfort->get_upshift_time_map();
        case C_DNTIME_MAP_ID:
            return comfort->get_downshift_time_map();
        case W_UPTIME_MAP_ID:
            return winter->get_upshift_time_map();
        case W_DNTIME_MAP_ID:
            return winter->get_downshift_time_map();
        case M_UPTIME_MAP_ID:
            return manual->get_upshift_time_map();
        case M_DNTIME_MAP_ID:
            return manual->get_downshift_time_map();
        case TCC_ADAPT_MAP_ID:
            return gearbox->tcc->tcc_learn_lockup_map;
        default:
            return nullptr;
    }
}

#define CHECK_MAP(map_id) \
    StoredTcuMap* ptr = get_map(map_id); \
    if (ptr == nullptr) { \
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT; \
    }

kwp_result_t MapEditor::read_map_data(uint8_t map_id, uint8_t read_type, uint16_t *dest_size_bytes, uint8_t** buffer) {
    CHECK_MAP(map_id)
    // Map valid
    uint16_t size = ptr->get_map_element_count();
    uint8_t* b = static_cast<uint8_t*>(heap_caps_malloc((size*sizeof(int16_t)), MALLOC_CAP_SPIRAM));
    if (b == nullptr) {
        ESP_LOGE("MAP_EDITOR_R", "Could not allocate read array!");
        return NRC_UN52_NO_MEM;
    }
    if (read_type ==  MAP_READ_TYPE_PRG) {
        memcpy(b, ptr->get_default_map_data(), size*sizeof(int16_t));
    } else if (read_type == MAP_READ_TYPE_MEM) {
        memcpy(b, ptr->get_current_data(), size*sizeof(int16_t));
    } else if (read_type == MAP_READ_TYPE_STO) {
        int16_t* eeprom_data = ptr->get_current_eeprom_map_data();
        memcpy(b, eeprom_data, size*sizeof(int16_t));
        free(eeprom_data);
    } else {
        free(buffer);
        return NRC_GENERAL_REJECT;
    }
    *buffer = b;
    *dest_size_bytes = size*sizeof(int16_t);
    return NRC_OK;
}

kwp_result_t MapEditor::read_map_metadata(uint8_t map_id, uint16_t *dest_size_bytes, uint8_t** buffer) {
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
    k_ptr = ptr->get_map_name();
    k_size = strlen(k_ptr);
    // 6 bytes for size data
    uint16_t size = 6+k_size+((x_size+y_size)*sizeof(int16_t));
    uint8_t* b = static_cast<uint8_t*>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM));
    if (b == nullptr) {
        return NRC_UN52_NO_MEM;
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
    return NRC_OK;
}
    
kwp_result_t MapEditor::write_map_data(uint8_t map_id, uint16_t dest_size, int16_t* buffer) {
    CHECK_MAP(map_id)
    if (ptr->replace_map_content(buffer, dest_size) == ESP_OK) {
        return NRC_OK;
    } else {
        return NRC_GENERAL_REJECT;
    }
}

kwp_result_t MapEditor::burn_to_eeprom(uint8_t map_id) {
    CHECK_MAP(map_id)
    if (ptr->save_to_eeprom() == ESP_OK) {
        return NRC_OK;
    } else {
        return NRC_GENERAL_REJECT;
    }
}

// uint8_t MapEditor::reset_to_program_default(uint8_t map_id) {
//     CHECK_MAP(map_id)
//     if (ptr->reload_from_eeprom() && ptr->save_to_eeprom()) {
//         return 0;
//     } else {
//         return NRC_GENERAL_REJECT;
//     }
// }

kwp_result_t MapEditor::undo_changes(uint8_t map_id) {
    CHECK_MAP(map_id)
    if (ptr->reload_from_eeprom() == ESP_OK) {
        return NRC_OK;
    } else {
        return NRC_GENERAL_REJECT;
    }
}
