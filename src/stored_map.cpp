#include "stored_map.h"
#include "nvs/eeprom_config.h"
#include "esp_heap_caps.h"

StoredTcuMap::StoredTcuMap( const char* eeprom_key_name,
    const uint16_t map_size,
    const int16_t* x_headers,
    const int16_t* y_headers,
    uint16_t x_size,
    uint16_t y_size,
    const int16_t* default_map)  :
    TcuMap(x_size,
    y_size,
    x_headers,
    y_headers){    
    this->initialized = false;
    this->default_map = default_map;
    if ((x_size*y_size) == map_size) {
        int16_t* dest = static_cast<int16_t*>(heap_caps_malloc(map_size*sizeof(int16_t), MALLOC_CAP_SPIRAM));
        if (nullptr != dest) {
            bool read_data_successful = EEPROM::read_nvs_map_data(eeprom_key_name, dest, default_map, map_size);
            if (read_data_successful) {
                bool data_added = this->add_data(dest, map_size);
                if (data_added) {
                    // Everything OK!
                    this->map_name = eeprom_key_name;
                    this->map_size = map_size;
                    this->initialized = true;
                }
                else {
                    ESP_LOGE("STO_MAP","Cannot Load Stored map %s! Map add data failed!", eeprom_key_name);
                }
            }
            else {
                ESP_LOGE("STO_MAP","Cannot Load Stored map %s! EEPROM Read NVS map failed!", eeprom_key_name);
            }
        }
        else {
            ESP_LOGE("STO_MAP","Cannot Load Stored map %s! Internal map allocation failed!", eeprom_key_name);
        }
        heap_caps_free(dest);        
    }
    else {
        ESP_LOGE("STO_MAP","Cannot Load Stored map %s! Map size is, but X and Y headers (%d,%d) make %d elements!", eeprom_key_name, x_size, y_size, x_size*y_size);        
    }    
}

bool StoredTcuMap::init_ok(void) const {
    return this->initialized;
}

/**
 * @brief Save new map contents to EEPROM (This will mean next TCU load will use the new map)
 */
bool StoredTcuMap::save_to_eeprom(void) {
    return EEPROM::write_nvs_map_data(this->map_name, this->get_current_data(), this->map_size); 
}

/**
 * @brief Replace map contents with new data (Keeping it in memory, call `save_to_eeprom` to write it to the TCU's EEPROM)
 * Note. This is a temporary replace. If you power the car down, changes made will be lost unless they
 * are written to EEPROM. This also acts as a failsafe in the event of a bad map edit, just reboot the car!
 */
bool StoredTcuMap::replace_map_content(int16_t* new_data, uint16_t content_len) {
    bool result = false;
    if (content_len == (this->map_size)) {
        result = this->add_data(new_data, this->map_size);
    } else {
        ESP_LOGE("STO_MAP", "replace_map_content failed! New data has invalid size of %d. Map should have %d entries", content_len, this->map_size);
    }
    return result;
}

/**
 * @brief Reloads the previously saved map from EEPROM into the map (Undo function)
 */
bool StoredTcuMap::reload_from_eeprom(void) {
    return this->read_from_eeprom(this->map_name, this->map_size);
}

/**
 * @brief Resets the map data to the stock map from the TCU firmware (maps.cpp)
 * THIS RESETS THE MAP TO FIRMWARE DEFAULT - ALL CHANGES WILL BE LOST!
 */
bool StoredTcuMap::reset_from_default_eeprom(void) {
    return this->add_data(const_cast<int16_t*>(this->default_map), this->map_size);
}

bool StoredTcuMap::read_from_eeprom(const char* key_name, uint16_t expected_size) {
    bool ret = false;
    bool mem_is_allocated = this->allocate_ok();
    if(mem_is_allocated) {
        int16_t* dest = static_cast<int16_t*>(heap_caps_malloc(expected_size* sizeof(int16_t), MALLOC_CAP_SPIRAM));
        if (dest != nullptr) {
            bool read_map_data_successful =  EEPROM::read_nvs_map_data(key_name, dest, this->default_map, expected_size);
            if (read_map_data_successful) {
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