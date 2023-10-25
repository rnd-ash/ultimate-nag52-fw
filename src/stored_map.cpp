#include "stored_map.h"
#include "nvs/eeprom_config.h"
#include "tcu_alloc.h"
#include "esp_check.h"

StoredMap::StoredMap(const char *eeprom_key_name,
                           const uint16_t data_element_count,
                           const int16_t *x_headers,
                           const int16_t *y_headers,
                           const uint16_t x_size,
                           const uint16_t y_size,
                           const int16_t *default_map) : LookupMap(x_headers,
                                                                x_size,
                                                                y_headers,
                                                                y_size,
                                                                default_map,
                                                                data_element_count)
{
    this->default_data = {0u};
    this->default_map = default_map;
    if ((x_size * y_size) == data_element_count)
    {
        int16_t *dest = static_cast<int16_t *>(TCU_HEAP_ALLOC(data_element_count * sizeof(int16_t)));
        if (nullptr != dest)
        {
            this->init_state = EEPROM::read_nvs_map_data(eeprom_key_name, dest, default_map, data_element_count);
            if (this->init_state == ESP_OK) {
                if (this->add_data(dest, data_element_count))
                {
                    // Everything OK!
		            this->data_element_count = data_element_count;
                    this->data_name = eeprom_key_name;
                    this->init_state = ESP_OK;
                }
                else
                {
                    ESP_LOGE("STO_MAP", "Cannot Load Stored map %s! Map add data failed!", eeprom_key_name);
                    this->init_state = ESP_ERR_INVALID_ARG; // Only if dest is nullptr
                }
            }
        }
        else
        {
            ESP_LOGE("STO_MAP", "Cannot Load Stored map %s! Internal map allocation failed!", eeprom_key_name);
            this->init_state = ESP_ERR_NO_MEM;
        }
        TCU_FREE(dest);
    }
    else
    {
        ESP_LOGE("STO_MAP", "Cannot Load Stored map %s! Map size is %d, but X and Y headers (%d,%d) make %d elements!", eeprom_key_name, data_element_count, x_size, y_size, x_size * y_size);
        this->init_state = ESP_ERR_INVALID_SIZE;
    }
}

/**
 * @brief Replace map contents with new data (Keeping it in memory, call `save_to_eeprom` to write it to the TCU's EEPROM)
 * Note. This is a temporary replace. If you power the car down, changes made will be lost unless they
 * are written to EEPROM. This also acts as a failsafe in the event of a bad map edit, just reboot the car!
 */
esp_err_t StoredMap::replace_data_content(const int16_t *new_data, uint16_t content_len)
{
    esp_err_t result = ESP_OK;
    if (content_len == (this->data_element_count))
    {
        if(!this->add_data(new_data, this->data_element_count)) {
            result = ESP_ERR_INVALID_STATE;
        }
    }
    else
    {
        result = ESP_ERR_NVS_INVALID_LENGTH;
    }
    return result;
}

esp_err_t StoredMap::reset_from_flash(void) {
    esp_err_t res = ESP_OK;
    const int16_t* default_data = this->default_map;
    if (ESP_OK == this->replace_data_content(default_data, this->dataSize) ) {
        res = this->save_to_eeprom();
    }
    return res;
}

/**
 * @brief Save new map contents to EEPROM (This will mean next TCU load will use the new map)
 */
esp_err_t StoredMap::save_to_eeprom(void)
{
    return EEPROM::write_nvs_map_data(this->data_name, this->get_current_data(), this->data_element_count);
}

esp_err_t StoredMap::read_from_eeprom(const char *key_name, uint16_t expected_size)
{
    esp_err_t ret;
    bool mem_is_allocated = this->is_allocated();
    if (mem_is_allocated)
    {
        int16_t *dest = static_cast<int16_t *>(TCU_HEAP_ALLOC(expected_size * sizeof(int16_t)));
        if (dest != nullptr)
        {
            ret = EEPROM::read_nvs_map_data(key_name, dest, this->default_map, expected_size);
            if (ret != ESP_OK)
            {
                if(!this->add_data(dest, expected_size)) {
                    ret = ESP_ERR_INVALID_ARG;
                }
            }
        }
        else
        {
            ESP_LOGE("STO_MAP", "Read from eeprom failed (Cannot allocate dest array)");
            ret = ESP_ERR_NO_MEM;
        }
        TCU_FREE(dest);
    }
    else
    {
        ESP_LOGE("STO_MAP", "Stored map has not been loaded! Internal map allocation failed!");
        ret = ESP_ERR_NO_MEM;
    }
    return ret;
}

uint16_t StoredMap::get_map_element_count(void)
{
    return this->data_element_count;
}

const int16_t *StoredMap::get_default_map_data(void)
{
    return this->default_map;
}

const char *StoredMap::get_map_name(void)
{
    return this->data_name;
}

int16_t *StoredMap::get_current_eeprom_map_data(void)
{
    bool succesful_allocation = false;
    int16_t *dest = static_cast<int16_t *>(TCU_HEAP_ALLOC(this->data_element_count * sizeof(int16_t)));
    if (nullptr != dest)
    {
        succesful_allocation = true;
        if (EEPROM::read_nvs_map_data(this->data_name, dest, this->default_map, this->data_element_count) != ESP_OK)
        {
            TCU_FREE(dest);
            succesful_allocation = false;            
        }
    }
    return succesful_allocation ? dest : nullptr;
}