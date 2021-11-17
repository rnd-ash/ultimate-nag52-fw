#include "eeprom_config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "scn.h"

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

bool EEPROM::init_eeprom() {
    // Called on startup

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE("EEPROM", "EEPROM init failed! %s", esp_err_to_name(err));
        // Don't erase flash. User has to do this manually. Just POST beep!
        return false;
    }
    nvs_handle_t config_handle;    err = nvs_open("Configuration", NVS_READWRITE, &config_handle);
    if (err != ESP_OK) {
        ESP_LOGE("EEPROM", "EEPROM NVS handle failed! %s", esp_err_to_name(err));
        return false;
    }

    uint16_t diff_ratio = 0;
    read_nvs_u16(config_handle, NVS_KEY_DIFF_RATIO, &diff_ratio, UINT16_MAX);
    return read_nvs_u16(config_handle, NVS_KEY_TCC_LOCKUP, &diff_ratio, 0);
}