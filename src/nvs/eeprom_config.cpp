#include "eeprom_config.h"
#include "esp_log.h"
#include "speaker.h"
#include <string.h>
#include "profiles.h"
#include "efuse/efuse.h"
#include "esp_efuse.h"
#include "esp_check.h"
#include "device_mode.h"
#include "esp_app_desc.h"
#include "all_keys.h"
#include "esp_flash.h"

uint16_t CURRENT_DEVICE_MODE = DEVICE_MODE_NORMAL;

esp_err_t EEPROM::read_nvs_map_data(const char* map_name, int16_t* dest, const int16_t* default_map, size_t map_element_count) {
    size_t byte_count = map_element_count*sizeof(int16_t);
    esp_err_t e = nvs_get_blob(MAP_NVS_HANDLE, map_name, dest, &byte_count);
    if (e == ESP_ERR_NVS_NOT_FOUND && default_map != nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "EEPROM", "Map %s not found in NVS. Setting to default map from prog flash", map_name);
        // Set default map data
        e = write_nvs_map_data(map_name, default_map, map_element_count);
        memcpy(dest, default_map, byte_count); // As e would be ESP_OK, the memcpy below won't get executed!
    }
    if(e != ESP_OK) {
        if (default_map != nullptr) {
            memcpy(dest, default_map, byte_count);
            e = ESP_OK;
        } else {
            e = ESP_ERR_INVALID_ARG;
        }
    } else {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "EEPROM", "Map %s loaded OK from NVS!", map_name);
    }
    return e;
}

esp_err_t EEPROM::check_if_new_fw(bool* dest) {
    esp_err_t res = ESP_OK;
    const esp_app_desc_t* now = esp_app_get_description();
    uint8_t sha[32];
    size_t len = 32;
    if (ESP_OK == nvs_get_blob(MAP_NVS_HANDLE, NVS_KEY_LAST_FW, &sha, &len)) {
        // Check and compare
        if (0 == memcmp(sha, now->app_elf_sha256, 32)) {
            *dest = false;
        } else {
            // Different
            nvs_set_blob(MAP_NVS_HANDLE, NVS_KEY_LAST_FW, now->app_elf_sha256, 32);
            *dest = true;
        }
    } else {
        res = nvs_set_blob(MAP_NVS_HANDLE, NVS_KEY_LAST_FW, now->app_elf_sha256, 32);
        *dest = true;
    }
    return res;
}

esp_err_t EEPROM::write_nvs_map_data(const char* map_name, const int16_t* to_write, size_t map_element_count) {
    esp_err_t e = nvs_set_blob(MAP_NVS_HANDLE, map_name, to_write, map_element_count*sizeof(int16_t));
    if (e != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error setting value for %s (%s)", map_name, esp_err_to_name(e));
    } else {
        e = nvs_commit(MAP_NVS_HANDLE);
        if (e != ESP_OK) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
        }
    }
    return e;
}

uint16_t EEPROM::read_device_mode(void) {
    uint16_t mode = 0;
    esp_err_t e = nvs_get_u16(MAP_NVS_HANDLE, NVS_KEY_DEV_MODE, &mode);
    if (ESP_OK != e) {
        if (e == ESP_ERR_NVS_NOT_FOUND) {
            mode |= DEVICE_MODE_NORMAL;
            e = EEPROM::set_device_mode(mode);
        } else {
            ESP_LOGE("EEPROM", "Device mode get failed: %s, returning normal", esp_err_to_name(e));
            mode |= DEVICE_MODE_NORMAL;
        }
    }
    return mode;
}

esp_err_t EEPROM::set_device_mode(uint16_t mode) {
    esp_err_t e = nvs_set_u16(MAP_NVS_HANDLE, NVS_KEY_DEV_MODE, mode);
    if (ESP_OK != e) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error setting device mode to %08X: %s", mode, esp_err_to_name(e));
    } else {
        e = nvs_commit(MAP_NVS_HANDLE);
        if (e != ESP_OK) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
        }
    }
    return e;
}

// bool read_nvs_gear_adaptation(nvs_handle_t handle, const char* key, pressure_map* map, size_t store_size) {
//     esp_err_t e = nvs_get_blob(handle, key, map, &store_size);
//     if (e == ESP_ERR_NVS_NOT_FOUND) {
//         ESP_LOG_LEVEL(ESP_LOG_WARN, "EEPROM", "Adaptation %s map not found. Creating a new one", key);
//         pressure_map new_map = {0,0,0,0,0,0,0,0,0,0,0};
//         e = nvs_set_blob(handle, key, &new_map, sizeof(new_map));
//         if (e != ESP_OK) {
//             ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error initializing default adaptation map map data (%s)", esp_err_to_name(e));
//             return false;
//         }
//         e = nvs_commit(handle);
//         if (e != ESP_OK) {
//             ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
//             return false;
//         }
//         ESP_LOG_LEVEL(ESP_LOG_INFO, "EEPROM", "New TCC map creation OK!");
//         memcpy(map, new_map, sizeof(new_map));
//         return true;
//     }
//     return (e == ESP_OK);
// }

esp_err_t EEPROM::init_eeprom() {
    // Called on startup
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "EEPROM init failed! %s", esp_err_to_name(result));
    } else {
        // Flash init OK
        nvs_handle_t config_handle;    
        result = nvs_open(NVS_PARTITION_USER_CFG, NVS_READWRITE, &config_handle);
        if (result != ESP_OK) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "EEPROM NVS handle failed! %s", esp_err_to_name(result));
        }
        MAP_NVS_HANDLE = config_handle;
        bool new_fw = false;
        if (ESP_OK == EEPROM::check_if_new_fw(&new_fw)) {
            if (new_fw) {
                ESP_LOGI("EEPROM", "New firmware after boot detected. Cleaning up NVS");
                nvs_iterator_t it = NULL;
                esp_err_t res = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
                while (ESP_OK == res) {
                    nvs_entry_info_t info;
                    nvs_entry_info(it, &info);
                    res = nvs_entry_next(&it);
                    bool found = false;
                    for (int i = 0; i < ALL_NVS_KEYS_LEN; i++) {
                        if (0 == strcmp(*ALL_NVS_KEYS[i], info.key)) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        ESP_LOGI("EEPROM", "Deleting NVS key %s\n", info.key);
                        nvs_erase_key(config_handle, info.key);
                    }
                }
                nvs_release_iterator(it);
                nvs_commit(config_handle);
                FLASH_NVS_SETTINGS_DESC desc = {};
                if (ESP_OK == esp_flash_read(esp_flash_default_chip, &desc, 0x330000, sizeof(FLASH_NVS_SETTINGS_DESC))) {
                    if (
                        desc.magic[0] == 0xDE &&
                        desc.magic[1] == 0xAD &&
                        desc.magic[2] == 0xBE &&
                        desc.magic[3] == 0xEF
                    ) {
                        // Valid data stored, check cs
                        // Also check flashed NVS config settings partition
                        uint32_t key_cs = 0;
                        for (int i = 0; i < ALL_NVS_KEYS_LEN; i++) {
                            const char* key = *ALL_NVS_KEYS[i];
                            int len = strlen(key);
                            for (int l = 0; l < len; l++) {
                                key_cs += l;
                                key_cs += key[l];
                            }
                        }
                        ESP_LOGI("EEPROM", "NVS CONFIG FLASH CHECK. CS OF KEYS: 0x%08X. CS OF FLASH: 0x%08X", (int)key_cs, (int)desc.key_cs);
                        if (key_cs != desc.key_cs) {
                            // Just erase 1 sector (Min), this way we avoid loads of writes, and config app will still be alerted
                            esp_flash_erase_region(esp_flash_default_chip, 0x330000, 4096);
                            ESP_LOGI("EEPROM", "Flash config desc erased");
                        } else {
                            ESP_LOGI("EEPROM", "NVS Flash config desc OK!");
                        }
                    } else {
                        ESP_LOGI("EEPROM", "No NVS Config data found");
                    }
                }
            }
        }
        result = read_core_config(&VEHICLE_CONFIG);
    }
    return result;
}

esp_err_t EEPROM::read_core_config(TCM_CORE_CONFIG* dest) {
    nvs_handle_t handle;
    nvs_open(NVS_PARTITION_USER_CFG, NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    size_t s = sizeof(TCM_CORE_CONFIG);
    esp_err_t result = nvs_get_blob(handle, NVS_KEY_CORE_SCN, dest, &s);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "EEPROM", "SCN Config not found. Creating a new one");
        TCM_CORE_CONFIG c = {
            .deprecated_is_large_nag = 0,
            .diff_ratio = 1000,
            .wheel_circumference = 2850,
            .is_four_matic = 0,
            .transfer_case_high_ratio = 1000,
            .transfer_case_low_ratio = 1000,
            .default_profile = 0, // Standard
            .red_line_rpm_diesel = 4500, // Safe for diesels, petrol-heads can change this!
            .red_line_rpm_petrol = 6000,
            .engine_type = 0,
            .egs_can_type = 0,
            .shifter_style = 0,
            .io_0_usage = 0,
            .input_sensor_pulses_per_rev = 1,
            .output_pulse_width_per_kmh = 1,
            .gen_mosfet_purpose = 0,
            .throttlevalve_maxopeningangle = 255, // 89.25°
            .c_eng = 1000,
            .engine_drag_torque = 400, // 40Nm
            .jeep_chrysler = false
        };
        result = nvs_set_blob(handle, NVS_KEY_CORE_SCN, &c, sizeof(c));
        if (result != ESP_OK) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error initializing default SCN config (%s)", esp_err_to_name(result));
        } else {
            // set blob OK
            result = nvs_commit(handle);
            if (result != ESP_OK) {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(result));
            } else {
                ESP_LOG_LEVEL(ESP_LOG_INFO, "EEPROM", "New SCN  creation OK!");
                memcpy(dest, &s, sizeof(s));
                result = ESP_OK;
            }
        }
        return true;
    }
    return result;
}

esp_err_t EEPROM::save_core_config(TCM_CORE_CONFIG* write) {
    nvs_handle_t handle;
    esp_err_t e;
    size_t s = sizeof(TCM_CORE_CONFIG);
    nvs_open(NVS_PARTITION_USER_CFG, NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    e = nvs_set_blob(handle, NVS_KEY_CORE_SCN, write, s);
    if (e != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error Saving SCN config (%s)", esp_err_to_name(e));
    } else {
        e = nvs_commit(handle);
        if (e != ESP_OK) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EEPROM", "Error calling nvs_commit: %s", esp_err_to_name(e));
        }
    }
    return e;
}

esp_err_t EEPROM::read_efuse_config(TCM_EFUSE_CONFIG* dest) {
    if (dest == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_BOARD_VER, &dest->board_ver, 8), "EFUSE_CFG", "Could not read board ver");
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_M_DAY, &dest->manufacture_day, 8), "EFUSE_CFG", "Could not read manf. day");
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_M_WEEK, &dest->manufacture_week, 8), "EFUSE_CFG", "Could not read manf. week");
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_M_MONTH, &dest->manufacture_month, 8), "EFUSE_CFG", "Could not read manf. month");
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_M_YEAR, &dest->manufacture_year, 8), "EFUSE_CFG", "Could not read manf. year");
    ESP_LOG_LEVEL(ESP_LOG_INFO, "EFUSE_CFG", "CONFIG:Board Ver: V1.%d, Week %d (%02d/%02d/%02d)", dest->board_ver, dest->manufacture_week, dest->manufacture_day, dest->manufacture_month, dest->manufacture_year);
    return ESP_OK;
}

esp_err_t EEPROM::write_efuse_config(TCM_EFUSE_CONFIG* dest) {
    if (dest == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->manufacture_month > 12 || dest->manufacture_month == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->manufacture_day > 31 || dest->manufacture_day == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->manufacture_week > 52 || dest->manufacture_week == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->manufacture_year < 22) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->board_ver == 0 || dest->board_ver > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_BOARD_VER, &dest->board_ver, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_M_DAY, &dest->manufacture_day, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_M_WEEK, &dest->manufacture_week, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_M_MONTH, &dest->manufacture_month, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_M_YEAR, &dest->manufacture_year, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
    
}

TCM_CORE_CONFIG VEHICLE_CONFIG = {};
TCM_EFUSE_CONFIG BOARD_CONFIG = {};
nvs_handle_t MAP_NVS_HANDLE = {};