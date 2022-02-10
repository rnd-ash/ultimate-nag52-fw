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

bool read_nvs_core_scn(nvs_handle_t handle, const char* key, EEPROM_CORE_SCN_CONFIG* scn, size_t store_size) {
    esp_err_t e = nvs_get_blob(handle, key, scn, &store_size);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("EEPROM", "SCN Config not found. Creating a new one");
        EEPROM_CORE_SCN_CONFIG s = {
            .is_large_nag = false,
            .diff_ratio = DIFF_RATIO,
            .wheel_diameter = TYRE_SIZE_MM,
            .is_four_matic = false,
            .transfer_case_high_ratio = 1000, // 1.0
            .transfer_case_low_ratio = 1000, // 1.0
        };
        e = nvs_set_blob(handle, key, &s, sizeof(s));
        if (e != ESP_OK) {
            ESP_LOGE("EEPROM", "Error initializing default SCN config (%s)", esp_err_to_name(e));
            return false;
        }
        e = nvs_commit(handle);
        if (e != ESP_OK) {
            ESP_LOGE("EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
            return false;
        }
        ESP_LOGI("EEPROM", "New TCC map creation OK!");
        memcpy(scn, &s, sizeof(s));
        return true;
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

bool read_nvs_tcc_adaptation(nvs_handle_t handle, const char* key, TccAdaptationData* store_location, size_t store_size) {
    esp_err_t e = nvs_get_blob(handle, key, store_location, &store_size);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("EEPROM", "TCC lockup map not found. Creating a new one");
        // Init default map
        TccAdaptationData new_map[NUM_GEARS];
        for (int i = 0; i < NUM_GEARS; i++) {
            for (int j = 0; j < 17; j++) {
                new_map[i].slip_values[j] = 250;
                new_map[i].learned[j] = false;
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
    /*
    save_nvs_gear_adaptation(NVS_KEY_1_2_ADAPTATION, &map_1_2_adaptation, sizeof(map_1_2_adaptation));
    save_nvs_gear_adaptation(NVS_KEY_2_3_ADAPTATION, &map_2_3_adaptation, sizeof(map_2_3_adaptation));
    save_nvs_gear_adaptation(NVS_KEY_3_4_ADAPTATION, &map_3_4_adaptation, sizeof(map_3_4_adaptation));
    save_nvs_gear_adaptation(NVS_KEY_4_5_ADAPTATION, &map_4_5_adaptation, sizeof(map_4_5_adaptation));
    */
    return true;
}

bool EEPROM::save_nvs_gear_adaptation(const char* key, pressure_map* read_location, size_t store_size) {
    nvs_handle_t handle;
    nvs_open("Configuration", NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    esp_err_t e = nvs_set_blob(handle, key, read_location, store_size);
    if (e != ESP_OK) {
        ESP_LOGE("EEPROM", "Error saving gear map data (%s)", esp_err_to_name(e));
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

bool read_core_scn_config(EEPROM_CORE_SCN_CONFIG* dest) {
    nvs_handle_t handle;
    nvs_open("Configuration", NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    return read_nvs_core_scn(handle, NVS_KEY_SCN_CONFIG, dest, sizeof(EEPROM_CORE_SCN_CONFIG));
}

bool save_core_scn_config(EEPROM_CORE_SCN_CONFIG* write) {
    nvs_handle_t handle;
    nvs_open("Configuration", NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    return false; // TODO
}

TccAdaptationData torque_converter_adaptation[NUM_GEARS];