#include "stored_map.h"
#include "nvs/eeprom_config.h"

StoredTcuMap::StoredTcuMap(
    const char* eeprom_key_name,
    const uint16_t map_size,
    const int16_t* x_headers,
    const int16_t* y_headers,
    int16_t x_size,
    int16_t y_size,
    const int16_t* default_map
) {
    this->internal = nullptr;
    if (x_size*y_size != map_size) {
        ESP_LOGE("STO_MAP","Cannot Load Stored map %s! Map size is, but X and Y headers (%d,%d) make %d elements!", eeprom_key_name, x_size, y_size, x_size*y_size);
        return;
    }
    this->internal = new TcmMap(x_size, y_size, static_cast<const int16_t*>(x_headers), static_cast<const int16_t*>(y_headers));
    if (!this->internal->allocate_ok()) {
        ESP_LOGE("STO_MAP","Cannot Load Stored map %s! Internal map allocation failed!", eeprom_key_name);
        delete this->internal;
        return;
    }

    if (!this->read_from_eeprom(eeprom_key_name, map_size)) {
        delete this->internal;
        return;
    }
    // Everything OK!
    this->map_name = eeprom_key_name;
    this->map_size = map_size;
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
        return false;
    } else {
        return this->internal->add_data(new_data, this->map_size);
    }
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
        delete[] dest;
        return false;
    }
    bool ret = this->internal->add_data(dest, expected_size);
    delete[] dest;
    return ret;
}

uint16_t StoredTcuMap::get_map_element_count() {
    return this->map_size;
}

const int16_t* StoredTcuMap::get_default_map_data() {
    return this->default_map;
}

int16_t* StoredTcuMap::get_current_map_data() {
    return this->internal->get_current_data();
}