#include "stored_table.h"
#include "nvs/eeprom_config.h"
#include "esp_heap_caps.h"
#include "esp_check.h"


StoredTable::StoredTable(const char * eeprom_key_name, const uint16_t data_element_count, const int16_t * x_headers, uint16_t x_size, const int16_t * default_data):
															LookupTable(x_headers,
                                                                x_size,
                                                                default_data,
                                                                data_element_count)
{
    this->default_data = default_data;
    if (x_size == data_element_count)
    {
        int16_t *dest = static_cast<int16_t *>(heap_caps_malloc(data_element_count * sizeof(int16_t), MALLOC_CAP_SPIRAM));
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
        heap_caps_free(dest);
    }
    else
    {
        ESP_LOGE("STO_MAP", "Cannot Load Stored map %s! Map size is %d, but has %d elements!", eeprom_key_name, x_size, data_element_count);
        this->init_state = ESP_ERR_INVALID_SIZE;
    }
}

esp_err_t StoredTable::read_from_eeprom(const char *key_name, uint16_t expected_size)
{
    esp_err_t ret;
    if (this->is_allocated())
    {
        int16_t *dest = static_cast<int16_t *>(heap_caps_malloc(expected_size * sizeof(int16_t), MALLOC_CAP_SPIRAM));
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
        heap_caps_free(dest);
    }
    else
    {
        ESP_LOGE("STO_MAP", "Stored map has not been loaded! Internal map allocation failed!");
        ret = ESP_ERR_NO_MEM;
    }
    return ret;
}