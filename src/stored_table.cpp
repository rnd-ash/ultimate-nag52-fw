#include "stored_table.h"
#include "nvs/eeprom_config.h"
#include "tcu_alloc.h"
#include "esp_check.h"


StoredTable::StoredTable(const char * eeprom_key_name, const uint16_t data_element_count, const int16_t * x_headers, uint16_t x_element_count, const int16_t * default_data):
															LookupTable(x_headers,
                                                                x_element_count,
                                                                default_data,
                                                                data_element_count)
{
    this->default_data = default_data;
    if (x_element_count == data_element_count)
    {
        int16_t *dest = static_cast<int16_t *>(TCU_HEAP_ALLOC(data_element_count * sizeof(int16_t)));
        if (nullptr != dest)
        {
            this->init_state = EEPROM::read_nvs_map_data(eeprom_key_name, dest, default_data, data_element_count);
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
        TCU_HEAP_FREE(dest);
    }
    else
    {
        ESP_LOGE("STO_MAP", "Cannot Load Stored map %s! Map size is %d, but has %d elements!", eeprom_key_name, x_element_count, data_element_count);
        this->init_state = ESP_ERR_INVALID_SIZE;
    }
}

esp_err_t StoredTable::read_from_eeprom(const char *key_name, uint16_t expected_size)
{
    esp_err_t ret;
    if (this->is_allocated())
    {
        int16_t *dest = static_cast<int16_t *>(TCU_HEAP_ALLOC(expected_size * sizeof(int16_t)));
        if (dest != nullptr)
        {
            ret = EEPROM::read_nvs_map_data(key_name, dest, this->default_data, expected_size);
			if (ESP_OK != ret)
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
        TCU_HEAP_FREE(dest);
    }
    else
    {
        ESP_LOGE("STO_MAP", "Stored map has not been loaded! Internal map allocation failed!");
        ret = ESP_ERR_NO_MEM;
    }
    return ret;
}

/**
 * @brief Save new map contents to EEPROM (This will mean next TCU load will use the new map)
 */
esp_err_t StoredTable::save_to_eeprom(void)
{
    return EEPROM::write_nvs_map_data(this->data_name, this->get_current_data(), this->data_element_count);
}

/**
 * @brief Replace map contents with new data (Keeping it in memory, call `save_to_eeprom` to write it to the TCU's EEPROM)
 * Note. This is a temporary replace. If you power the car down, changes made will be lost unless they
 * are written to EEPROM. This also acts as a failsafe in the event of a bad map edit, just reboot the car!
 */
esp_err_t StoredTable::replace_data_content(int16_t *new_data, uint16_t content_len)
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
