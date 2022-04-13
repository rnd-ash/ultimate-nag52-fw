#include "eeprom_config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "scn.h"
#include "speaker.h"
#include <string.h>
#include "profiles.h"

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

bool read_nvs_gear_adaptation(nvs_handle_t handle, const char* key, pressure_map* map, size_t store_size) {
    esp_err_t e = nvs_get_blob(handle, key, map, &store_size);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("EEPROM", "Adaptation %s map not found. Creating a new one", key);
        pressure_map new_map = {0,0,0,0,0,0,0,0,0,0,0};
        e = nvs_set_blob(handle, key, &new_map, sizeof(new_map));
        if (e != ESP_OK) {
            ESP_LOGE("EEPROM", "Error initializing default adaptation map map data (%s)", esp_err_to_name(e));
            return false;
        }
        e = nvs_commit(handle);
        if (e != ESP_OK) {
            ESP_LOGE("EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
            return false;
        }
        ESP_LOGI("EEPROM", "New TCC map creation OK!");
        memcpy(map, new_map, sizeof(new_map));
        return true;
    }
    return (e == ESP_OK);
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
    err = nvs_open(NVS_PARTITION_USER_CFG, NVS_READWRITE, &config_handle);
    if (err != ESP_OK) {
        ESP_LOGE("EEPROM", "EEPROM NVS handle failed! %s", esp_err_to_name(err));
        return false;
    }
    bool res = read_core_config(&VEHICLE_CONFIG);
    if (!res) {
        ESP_LOGE("EEPROM", "EEPROM SCN config read failed! %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool EEPROM::read_core_config(TCM_CORE_CONFIG* dest) {
    nvs_handle_t handle;
    nvs_open(NVS_PARTITION_USER_CFG, NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    size_t s = sizeof(TCM_CORE_CONFIG);
    esp_err_t e = nvs_get_blob(handle, NVS_KEY_SCN_CONFIG, dest, &s);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("EEPROM", "SCN Config not found. Creating a new one");
        TCM_CORE_CONFIG s = {
#ifdef LARGE_NAG
            .is_large_nag = 1,
#else
            .is_large_nag = 0,
#endif
            .diff_ratio = DIFF_RATIO,
            .wheel_circumference = TYRE_SIZE_MM,
#ifdef FOUR_MATIC
            .is_four_matic = 1,
            .transfer_case_high_ratio = TC_RATIO_HIGH,
            .transfer_case_low_ratio = TC_RATIO_LOW,
#else
            .is_four_matic = 0,
            .transfer_case_high_ratio = 1000,
            .transfer_case_low_ratio = 1000,
#endif
            .default_profile = 0, // Standard
            .red_line_rpm_diesel = 4500, // Safe for diesels, petrol-heads can change this!
            .red_line_rpm_petrol = 6000,
            .engine_type = 0 // Diesel by default - TODO We should read this via CAN
        };
        e = nvs_set_blob(handle, NVS_KEY_SCN_CONFIG, &s, sizeof(s));
        if (e != ESP_OK) {
            ESP_LOGE("EEPROM", "Error initializing default SCN config (%s)", esp_err_to_name(e));
            return false;
        }
        e = nvs_commit(handle);
        if (e != ESP_OK) {
            ESP_LOGE("EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
            return false;
        }
        ESP_LOGI("EEPROM", "New SCN  creation OK!");
        memcpy(dest, &s, sizeof(s));
        return true;
    } else if (e != ESP_OK) {
        ESP_LOGE("EEPROM", "Could not read SCN config: %s", esp_err_to_name(e));
    }
    return (e == ESP_OK);
}

bool EEPROM::save_core_config(TCM_CORE_CONFIG* write) {
    nvs_handle_t handle;
    esp_err_t e;
    size_t s = sizeof(TCM_CORE_CONFIG);
    nvs_open(NVS_PARTITION_USER_CFG, NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    e = nvs_set_blob(handle, NVS_KEY_SCN_CONFIG, write, s);
    if (e != ESP_OK) {
        ESP_LOGE("EEPROM", "Error Saving SCN config (%s)", esp_err_to_name(e));
        return false;
    }
    e = nvs_commit(handle);
    if (e != ESP_OK) {
        ESP_LOGE("EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
        return false;
    }
    return true;
}

TCM_CORE_CONFIG VEHICLE_CONFIG = {};