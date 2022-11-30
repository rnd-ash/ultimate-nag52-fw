#include "stored_map.h"
#include "nvs/eeprom_config.h"
#include "esp_heap_caps.h"

StoredTcuMap::StoredTcuMap(
    const char* eeprom_key_name,
    const uint16_t map_element_count,
    const int16_t* x_headers,
    const int16_t* y_headers,
    uint16_t x_size,
    uint16_t y_size,
    const int16_t* default_map
) : TcuMap(x_size, y_size, x_headers, y_headers) {
    if (x_size*y_size != map_element_count) {
        ESP_LOGE("STO_MAP","Cannot Load Stored map %s! Map size is, but X and Y headers (%d,%d) make %d elements!", eeprom_key_name, x_size, y_size, x_size*y_size);
        return;
    }
    this->default_map = default_map;
    this->map_name = eeprom_key_name;
    this->map_element_count = map_element_count;
    if (!this->read_from_eeprom(eeprom_key_name, map_element_count)) {
        return;
    }
}

bool StoredTcuMap::init_ok(void) const {
    return this->initialized;
}

/**
 * @brief Save new map contents to EEPROM (This will mean next TCU load will use the new map)
 */
bool StoredTcuMap::save_to_eeprom() {
    return EEPROM::write_nvs_map_data(this->map_name, this->get_current_data(), this->map_element_count); 
}

/**
 * @brief Replace map contents with new data (Keeping it in memory, call `save_to_eeprom` to write it to the TCU's EEPROM)
 * Note. This is a temporary replace. If you power the car down, changes made will be lost unless they
 * are written to EEPROM. This also acts as a failsafe in the event of a bad map edit, just reboot the car!
 */
bool StoredTcuMap::replace_map_content(int16_t* new_data, uint16_t content_len) {
    if (content_len != this->map_element_count) {
        ESP_LOGE("STO_MAP", "replace_map_content failed! New data has invalid size of %d. Map should have %d entries", content_len, this->map_element_count);
        return false;
    } else {
        return this->add_data(new_data, this->map_element_count);
    }
}

/**
 * @brief Reloads the previously saved map from EEPROM into the map (Undo function)
 */
bool StoredTcuMap::reload_from_eeprom() {
    return this->read_from_eeprom(this->map_name, this->map_element_count);
}

/**
 * @brief Resets the map data to the stock map from the TCU firmware (maps.cpp)
 * THIS RESETS THE MAP TO FIRMWARE DEFAULT - ALL CHANGES WILL BE LOST!
 */
bool StoredTcuMap::reset_from_default_eeprom() {
    return this->add_data((int16_t*)this->default_map, this->map_element_count);
}

bool StoredTcuMap::read_from_eeprom(const char* key_name, uint16_t expected_size) {
    bool ret = false;
    bool mem_is_allocated = this->allocate_ok();
    if(mem_is_allocated) {
        int16_t* dest = static_cast<int16_t*>(heap_caps_malloc(expected_size* sizeof(int16_t), MALLOC_CAP_SPIRAM));
        if (dest != nullptr) {
            if (EEPROM::read_nvs_map_data(key_name, dest, this->default_map, expected_size)) {
                ret = this->add_data(dest, expected_size);
            }
            else {
                ESP_LOGE("STO_MAP", "Read from eeprom failed (read_nvs_map_data failed)");
            }
        }
        else {
            ESP_LOGE("STO_MAP", "Read from eeprom failed (Cannot allocate dest array)");            
        }
        heap_caps_free(dest);
    }
    else {
        ESP_LOGE("STO_MAP","Stored map has not been loaded! Internal map allocation failed!");
    }
    return ret;
}

uint16_t StoredTcuMap::get_map_element_count() {
    return this->map_element_count;
}

const int16_t* StoredTcuMap::get_default_map_data() {
    return this->default_map;
}

const char* StoredTcuMap::get_map_name() {
    return this->map_name;
}

int16_t* StoredTcuMap::get_current_eeprom_map_data() {
    int16_t* dest = static_cast<int16_t*>(heap_caps_malloc(this->map_element_count * sizeof(int16_t), MALLOC_CAP_SPIRAM));
    if (dest == nullptr) {
        return nullptr;
    }
    if (EEPROM::read_nvs_map_data(this->map_name, dest, this->default_map, this->map_element_count)) {
        return dest;
    } else {
        free(dest);
        return nullptr;
    }
}