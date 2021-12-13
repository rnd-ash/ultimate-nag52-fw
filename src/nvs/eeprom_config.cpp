#include "eeprom_config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "scn.h"
    #include "speaker.h"

// dest HAS TO BE ALLOCATED!
bool read_map_data(nvs_handle_t handle, const char* map_name, SHIFT_LAYER* dest) {
    size_t size = sizeof(SHIFT_LAYER);
    esp_err_t e = nvs_get_blob(handle, map_name, dest, &size);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("EEPROM", "Map table for %s not found", map_name);
        e = nvs_set_blob(handle, map_name, dest, sizeof(SHIFT_LAYER));
        e = nvs_commit(handle);
    }
    return (e == ESP_OK);
}

// src HAS TO BE ALLOCATED
bool write_map_data(nvs_handle_t handle, const char* map_name, SHIFT_LAYER* src) {
    esp_err_t w = nvs_set_blob(handle, map_name, src, sizeof(SHIFT_LAYER));
    if (w != ESP_OK) {
        ESP_LOGE("EEPROM", "Could not write map data for %s: %s", map_name, esp_err_to_name(w));
        return false;
    }
    return true;
}

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
    nvs_handle_t config_handle;    
    err = nvs_open("Configuration", NVS_READWRITE, &config_handle);
    if (err != ESP_OK) {
        ESP_LOGE("EEPROM", "EEPROM NVS handle failed! %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

#define ALLOCATE_READ_TIMING_MAP(handle, map, name) \
    map = (SHIFT_LAYER*)malloc(sizeof(SHIFT_LAYER)); \
    if (map == nullptr) { \
        ESP_LOGE("EEPROM", "Allocation for timing map %s failed!", name); \
        return false; \
    } \
    if (!read_map_data(handle, name, map)) { \
        return false; \
    }

bool EEPROM::load_map_data() {
    nvs_handle_t timing_handle;
    esp_err_t err = nvs_flash_init_partition("tcm_map");
    err = nvs_open_from_partition("tcm_map", "TimingData", NVS_READWRITE, &timing_handle);
    if (err != ESP_OK) {
        ESP_LOGE("EEPROM", "EEPROM NVS handle failed! %s", esp_err_to_name(err));
        return false;
    }
    // Upshift timings (Alloc)
    ALLOCATE_READ_TIMING_MAP(timing_handle, one_to_two, NVS_KEY_1_2)
    ALLOCATE_READ_TIMING_MAP(timing_handle, two_to_three, NVS_KEY_2_3)
    ALLOCATE_READ_TIMING_MAP(timing_handle, three_to_four, NVS_KEY_3_4)
    ALLOCATE_READ_TIMING_MAP(timing_handle, four_to_five, NVS_KEY_4_5)

    // Downshifts timings (Alloc)
    ALLOCATE_READ_TIMING_MAP(timing_handle, two_downto_one, NVS_KEY_2_1)
    ALLOCATE_READ_TIMING_MAP(timing_handle, three_downto_two, NVS_KEY_3_2)
    ALLOCATE_READ_TIMING_MAP(timing_handle, four_downto_three, NVS_KEY_4_3)
    ALLOCATE_READ_TIMING_MAP(timing_handle, five_downto_four, NVS_KEY_5_4)

    nvs_stats_t stats;
    nvs_get_stats("tcm_map", &stats);
    ESP_LOGI("EEPROM", "Entry stats: used: %d total: %d free: %d", stats.used_entries, stats.total_entries, stats.free_entries);

    return true;
}

bool EEPROM::save_map_data() {
    nvs_handle_t timing_handle;   
    esp_err_t err = nvs_flash_init_partition("tcm_map"); 
    err = nvs_open_from_partition("tcm_map", "TimingData", NVS_READWRITE, &timing_handle);
    if (err != ESP_OK) {
        ESP_LOGE("EEPROM", "EEPROM NVS handle failed! %s", esp_err_to_name(err));
        return false;
    }
    // Commit changes
    write_map_data(timing_handle, NVS_KEY_1_2, one_to_two);
    write_map_data(timing_handle, NVS_KEY_2_3, two_to_three);
    write_map_data(timing_handle, NVS_KEY_3_4, three_to_four);
    write_map_data(timing_handle, NVS_KEY_4_5, four_to_five);

    write_map_data(timing_handle, NVS_KEY_2_1, two_downto_one);
    write_map_data(timing_handle, NVS_KEY_3_2, three_downto_two);
    write_map_data(timing_handle, NVS_KEY_4_3, four_downto_three);
    write_map_data(timing_handle, NVS_KEY_5_4, five_downto_four);

    nvs_commit(timing_handle);
    spkr.send_note(1000, 250, 250);
    return true;
}

SHIFT_LAYER* one_to_two = nullptr;
SHIFT_LAYER* two_to_three = nullptr;
SHIFT_LAYER* three_to_four = nullptr;
SHIFT_LAYER* four_to_five = nullptr;

SHIFT_LAYER* five_downto_four = nullptr;
SHIFT_LAYER* four_downto_three = nullptr;
SHIFT_LAYER* three_downto_two = nullptr;
SHIFT_LAYER* two_downto_one = nullptr;