#include "stored_data.h"
#include "esp_heap_caps.h"
#include "nvs/eeprom_config.h"

esp_err_t StoredData::init_status(void) const
{
    return this->init_state;
}

/**
 * @brief Resets the map data to the stock map from the TCU firmware (maps.cpp)
 * THIS RESETS THE MAP TO FIRMWARE DEFAULT - ALL CHANGES WILL BE LOST!
 */
// bool StoredData::reset_from_default_eeprom(void)
// {
//     return this->add_data(const_cast<int16_t *>(this->default_data), this->map_element_count);
// }

uint16_t StoredData::get_data_element_count(void)
{
    return this->data_element_count;
}

const int16_t *StoredData::get_default_data(void)
{
    return this->default_data;
}

const char *StoredData::get_data_name(void)
{
    return this->data_name;
}

int16_t *StoredData::get_current_eeprom_data(void)
{
    bool succesful_allocation = false;
    int16_t *dest = static_cast<int16_t *>(heap_caps_malloc(this->data_element_count * sizeof(int16_t), MALLOC_CAP_SPIRAM));
    if (nullptr != dest)
    {
        succesful_allocation = true;
        if (EEPROM::read_nvs_map_data(this->data_name, dest, this->default_data, this->data_element_count) != ESP_OK)
        {
            heap_caps_free(dest);
            succesful_allocation = false;            
        }
    }
    return succesful_allocation ? dest : nullptr;
}

/**
 * @brief Reloads the previously saved map from EEPROM into the map (Undo function)
 */
esp_err_t StoredData::reload_from_eeprom(void)
{
    return this->read_from_eeprom(this->data_name, this->data_element_count);
}
