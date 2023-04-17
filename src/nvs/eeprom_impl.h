/** @file */
#ifndef EEPROM_IMPL_H
#define EEPROM_IMPL_H

#include "eeprom_config.h"

namespace EEPROM {
    template <typename T>
    esp_err_t read_subsystem_settings(const char* key_name, T* dest, const T* default_settings) {
        size_t size = sizeof(T);
        esp_err_t e = nvs_get_blob(MAP_NVS_HANDLE, key_name, dest, &size);
        if (e == ESP_ERR_NVS_NOT_FOUND && key_name != nullptr) {
            ESP_LOG_LEVEL(ESP_LOG_WARN, "EEPROM", "subsystem %s not found in NVS. Setting to settings from prog flash", key_name);
            // Set default map data
            e = write_subsystem_settings(key_name, default_settings);
            memcpy(dest, default_settings, sizeof(T)); // As e would be ESP_OK, the memcpy below won't get executed!
        }
        if(e != ESP_OK) {
            if (default_settings != nullptr) {
                memcpy(dest, default_settings, sizeof(T));
                e = ESP_OK;
            } else {
                e = ESP_ERR_INVALID_ARG;
            }
        } else {
            ESP_LOG_LEVEL(ESP_LOG_INFO, "EEPROM", "subsystem %s loaded OK from NVS!", key_name);
        }
        return e;
    }

    template <typename T>
    esp_err_t write_subsystem_settings(const char* key_name, const T* write) {
        esp_err_t e = nvs_set_blob(MAP_NVS_HANDLE, key_name, write, sizeof(T));
        if (e != ESP_OK) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error writing subsystem settings for %s (%s)", key_name, esp_err_to_name(e));
        } else {
            e = nvs_commit(MAP_NVS_HANDLE);
            if (e != ESP_OK) {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
            }
        }
        return e;
    }

}
#endif