#include "module_settings.h"
#include "eeprom_impl.h"

TCC_MODULE_SETTINGS TCC_CURRENT_SETTINGS = TCC_DEFAULT_SETTINGS;
SOL_MODULE_SETTINGS SOL_CURRENT_SETTINGS = SOL_DEFAULT_SETTINGS;
SBS_MODULE_SETTINGS SBS_CURRENT_SETTINGS = SBS_DEFAULT_SETTINGS;

// These macro will fail should the naming convension of the settings not be correct
// so it enforces the following rule:

// (xxx) denotes the 3 letter prefix of the module settings name
//          NVS Key name : xxx_SETTINGS_NVS_KEY
// Current settings name : xxx_CURRENT_SETTINGS
// Default settings name : xxx_DEFAULT_SETTINGS
// Settings variable type: xxx_MODULE_SETTINGS
// Settings SCN KEY IDs  : xxx_MODULE_SETINGS_SCN_ID
#define READ_EEPROM_SETTING(pfx) \
    EEPROM::read_subsystem_settings<pfx##_MODULE_SETTINGS>(pfx##_SETTINGS_NVS_KEY, &pfx##_CURRENT_SETTINGS, &pfx##_DEFAULT_SETTINGS)

#define RESET_EEPROM_SETINGS(pfx) \
        pfx##_CURRENT_SETTINGS = pfx##_DEFAULT_SETTINGS; \
        return EEPROM::write_subsystem_settings(pfx##_SETTINGS_NVS_KEY, &pfx##_DEFAULT_SETTINGS); \

// Checks and writes the buffer as the setting
#define CHECK_AND_WRITE_SETTINGS(pfx, buffer_len, buffer) \
    if (sizeof(pfx##_MODULE_SETTINGS) != buffer_len) { \
        return ESP_ERR_INVALID_SIZE; \
    } else { \
        pfx##_MODULE_SETTINGS settings = *(pfx##_MODULE_SETTINGS*)buffer; \
        pfx##_CURRENT_SETTINGS = settings; \
        return EEPROM::write_subsystem_settings(pfx##_SETTINGS_NVS_KEY, &pfx##_CURRENT_SETTINGS); \
    } \

#define READ_SETTINGS_TO_BUFFER(pfx, buffer_len_dest, buffer_dest) \
    const pfx##_MODULE_SETTINGS* ptr = &pfx##_CURRENT_SETTINGS; \
    uint8_t* dest = (uint8_t*)heap_caps_malloc(sizeof(pfx##_MODULE_SETTINGS)+1, MALLOC_CAP_SPIRAM); \
    if (nullptr == ptr || nullptr == dest) { \
        delete[] dest; \
        return ESP_ERR_NO_MEM; \
    } else { \
        dest[0] = pfx##_MODULE_SETINGS_SCN_ID; \
        memcpy(&dest[1], ptr, sizeof(pfx##_MODULE_SETTINGS)); \
        *buffer_len = sizeof(pfx##_MODULE_SETTINGS)+1; \
        *buffer = dest; \
        return ESP_OK; \
    } \

esp_err_t ModuleConfiguration::load_all_settings() {
    esp_err_t res = ESP_OK;
    READ_EEPROM_SETTING(TCC); // Torque converter
    READ_EEPROM_SETTING(SOL); // Solenoid program
    READ_EEPROM_SETTING(SBS); // Shift basic control program
    return res;
}

esp_err_t ModuleConfiguration::reset_settings(uint8_t idx) {
    switch (idx) {
        case TCC_MODULE_SETINGS_SCN_ID:
            RESET_EEPROM_SETINGS(TCC);
            break;
        case SOL_MODULE_SETINGS_SCN_ID:
            RESET_EEPROM_SETINGS(SOL);
            break;
        case SBS_MODULE_SETINGS_SCN_ID:
            RESET_EEPROM_SETINGS(SBS);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t ModuleConfiguration::read_settings(uint8_t module_id, uint16_t* buffer_len, uint8_t** buffer) {
    if (module_id == TCC_MODULE_SETINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(TCC, buffer_len, buffer);
    } else if (module_id == SOL_MODULE_SETINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(SOL, buffer_len, buffer);
    } else if (module_id == SBS_MODULE_SETINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(SBS, buffer_len, buffer);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t ModuleConfiguration::write_settings(uint8_t module_id, uint16_t buffer_len, uint8_t* buffer) {
    if (module_id == TCC_MODULE_SETINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(TCC, buffer_len, buffer)
    } else if (module_id == SOL_MODULE_SETINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(SOL, buffer_len, buffer)
    }  else if (module_id == SBS_MODULE_SETINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(SBS, buffer_len, buffer)
    }
    return ESP_ERR_INVALID_ARG;
}