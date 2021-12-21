#include "eeprom_config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "scn.h"
#include "speaker.h"
#include <string.h>


/// If default is UINT16_MAX, it is ignored
bool read_nvs_u16(nvs_handle_t handle, const char* key, uint16_t* ptr, uint16_t default_value) {
    esp_err_t e = nvs_get_u16(handle, key, ptr);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        if (default_value == UINT16_MAX) {
            ESP_LOGW("EEPROM", "Value for %s not found, no defaults specified. CANNOT CONTINUE!", key);
            return false;
        } else {
            // Try to init the default value
            ESP_LOGW("EEPROM", "Value for %s not found, initializing to default of %u", key, default_value);
            e = nvs_set_u16(handle, key, default_value);
            if (e != ESP_OK) {
                ESP_LOGE("EEPROM", "Error setting default value for %s (%s)", key, esp_err_to_name(e));
                return false;
            }
            e = nvs_commit(handle);
            if (e != ESP_OK) {
                ESP_LOGE("EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
                return false;
            }
            // OK
            *ptr = default_value;
            return true;
        }
    }
    return (e == ESP_OK);
}

bool read_nvs_tcc_adaptation(nvs_handle_t handle, const char* key, TccAdaptationData* store_location, size_t store_size) {
    esp_err_t e = nvs_get_blob(handle, key, store_location, &store_size);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("EEPROM", "TCC lockup map not found. Creating a new one");
        // Init default map
        TccAdaptationData new_map[NUM_GEARS];
        for (int i = 0; i < NUM_GEARS; i++) {
            new_map[i].lock_rpm_limit = 150;
            new_map[i].slip_rpm_limit = 250;
            for (int j = 0; j < 17; j++) {
                new_map[i].slip_values[j] = 0;
                new_map[i].lockup_values[j] = 0;
            }
        }
        e = nvs_set_blob(handle, key, &new_map, sizeof(new_map));
        if (e != ESP_OK) {
            ESP_LOGE("EEPROM", "Error initializing default TCC map data (%s)", esp_err_to_name(e));
            return false;
        }
        e = nvs_commit(handle);
        if (e != ESP_OK) {
            ESP_LOGE("EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
            return false;
        }
        ESP_LOGI("EEPROM", "New TCC map creation OK!");
        memcpy(store_location, new_map, sizeof(new_map));
        return true;
    }
    return (e == ESP_OK);
}

bool EEPROM::save_nvs_tcc_adaptation(TccAdaptationData* read_location, size_t store_size) {
    nvs_handle_t handle;
    nvs_open("Configuration", NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    esp_err_t e = nvs_set_blob(handle, NVS_KEY_TCC_ADAPTATION, read_location, store_size);
    if (e != ESP_OK) {
        ESP_LOGE("EEPROM", "Error initializing default TCC map data (%s)", esp_err_to_name(e));
        return false;
    }
    e = nvs_commit(handle);
    if (e != ESP_OK) {
        ESP_LOGE("EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
        return false;
    }
    return true;
}

bool EEPROM::init_eeprom() {
    // Called on startup

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE("EEPROM", "EEPROM init failed! %s", esp_err_to_name(err));
        // Don't erase flash. User has to do this manually. Just POST beep!
        return false;
    }
    nvs_handle_t config_handle;    
    err = nvs_open("Configuration", NVS_READWRITE, &config_handle);
    if (err != ESP_OK) {
        ESP_LOGE("EEPROM", "EEPROM NVS handle failed! %s", esp_err_to_name(err));
        return false;
    }
    if (read_nvs_tcc_adaptation(config_handle, NVS_KEY_TCC_ADAPTATION, torque_converter_adaptation, (size_t)sizeof(torque_converter_adaptation)) != true) {
        return false;
    }
    return true;
}

TccAdaptationData torque_converter_adaptation[NUM_GEARS];