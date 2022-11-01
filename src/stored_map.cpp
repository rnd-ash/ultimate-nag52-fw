#include "stored_map.h"
#include "nvs/eeprom_config.h"

StoredTcuMap::StoredTcuMap(
    const char* eeprom_key_name,
    const uint16_t map_size,
    int16_t* x_headers,
    int16_t* y_headers,
    const int16_t* default_map
) {

}

bool StoredTcuMap::init_ok() {
    return this->internal != nullptr;
}

/**
 * @brief Save new map contents to EEPROM (This will mean next TCU load will use the new map)
 */
bool StoredTcuMap::save_to_eeprom() {
    return EEPROM::write_nvs_map_data(this->map_name, internal->get_current_data(), this->map_size); 
}

/**
 * @brief Replace map contents with new data (Keeping it in memory, call `save_to_eeprom` to write it to the TCU's EEPROM)
 * Note. This is a temporary replace. If you power the car down, changes made will be lost unless they
 * are written to EEPROM. This also acts as a failsafe in the event of a bad map edit, just reboot the car!
 */
bool StoredTcuMap::replace_map_content(int16_t* new_data, uint16_t content_len) {
    if (content_len != this->map_size) {
        ESP_LOGE("STO_MAP", "replace_map_content failed! New data has invalid size of %d. Map should have %d entries", content_len, this->map_size);
    } else {
        return this->internal->add_data(new_data, this->map_size);
    }
    return false;
}

/**
 * @brief Reloads the previously saved map from EEPROM into the map (Undo function)
 */
bool StoredTcuMap::reload_from_eeprom() {
    return this->read_from_eeprom(this->map_name, this->map_size);
}

/**
 * @brief Resets the map data to the stock map from the TCU firmware (maps.cpp)
 * THIS RESETS THE MAP TO FIRMWARE DEFAULT - ALL CHANGES WILL BE LOST!
 */
bool StoredTcuMap::reset_from_default_eeprom() {
    return this->internal->add_data((int16_t*)this->default_map, this->map_size);
}

// See TcmMap.cpp
float StoredTcuMap::get_value(float x_value, float y_value) {
    return this->internal->get_value(x_value, y_value);
}

bool StoredTcuMap::read_from_eeprom(const char* key_name, uint16_t expected_size) {
    if (this->internal == nullptr) {
        return false;
    }
    int16_t* dest = (int16_t*)malloc(expected_size* sizeof(int16_t));
    if (dest == nullptr) {
        ESP_LOGE("STO_MAP", "Read from eeprom failed (Cannot allocate dest array)");
        return false;
    }
    if (!EEPROM::read_nvs_map_data(key_name, dest, this->default_map, expected_size)) {
        ESP_LOGE("STO_MAP", "Read from eeprom failed (read_nvs_map_data failed)");
        return false;
    }
    return this->internal->add_data(dest, expected_size);
}