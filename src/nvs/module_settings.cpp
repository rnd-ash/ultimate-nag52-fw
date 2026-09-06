#include "module_settings.h"
#include "eeprom_impl.h"
#include "tcu_alloc.h"
#include "all_keys.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

namespace {

/**
 * @brief Validates a settings block that has just arrived over the diagnostic
 *        link, before it is applied and persisted to NVS.
 *
 * By default any block is accepted. Specialize for modules that contain values
 * the runtime cannot defend itself against (divisors, and anything a NaN would
 * poison), so a bad write is rejected rather than bricking the TCU across a
 * reboot.
 */
template <typename T>
bool sanitize_settings(T*) {
    return true;
}

template <>
bool sanitize_settings<SOL_MODULE_SETTINGS>(SOL_MODULE_SETTINGS* settings) {
    // cc_reference_resistance is a divisor in the constant current solenoid
    // controller, and cc_vref_solenoid is the numerator.
    return isfinite(settings->cc_reference_resistance) &&
        settings->cc_reference_resistance > 0.1f &&
        settings->cc_reference_resistance < 1000.0f &&
        isfinite(settings->cc_reference_temp) &&
        isfinite(settings->cc_temp_coefficient_wires) &&
        settings->cc_vref_solenoid != 0u;
}

} // namespace

TCC_MODULE_SETTINGS TCC_CURRENT_SETTINGS = TCC_DEFAULT_SETTINGS;
SOL_MODULE_SETTINGS SOL_CURRENT_SETTINGS = SOL_DEFAULT_SETTINGS;
SBS_MODULE_SETTINGS SBS_CURRENT_SETTINGS = SBS_DEFAULT_SETTINGS;
PRM_MODULE_SETTINGS PRM_CURRENT_SETTINGS = PRM_DEFAULT_SETTINGS;
ADP_MODULE_SETTINGS ADP_CURRENT_SETTINGS = ADP_DEFAULT_SETTINGS;
ETS_MODULE_SETTINGS ETS_CURRENT_SETTINGS = ETS_DEFAULT_SETTINGS;
REL_MODULE_SETTINGS REL_CURRENT_SETTINGS = REL_DEFAULT_SETTINGS;
GAR_MODULE_SETTINGS GAR_CURRENT_SETTINGS = GAR_DEFAULT_SETTINGS;
CRS_MODULE_SETTINGS CRS_CURRENT_SETTINGS = CRS_DEFAULT_SETTINGS;

// These macro will fail should the naming convension of the settings not be correct
// so it enforces the following rule:

// (xxx) denotes the 3 letter prefix of the module settings name
//          NVS Key name : xxx_SETTINGS_NVS_KEY
// Current settings name : xxx_CURRENT_SETTINGS
// Default settings name : xxx_DEFAULT_SETTINGS
// Settings variable type: xxx_MODULE_SETTINGS
// Settings SCN KEY IDs  : xxx_MODULE_SETTINGS_SCN_ID
#define READ_EEPROM_SETTING(pfx) \
    EEPROM::read_subsystem_settings<pfx##_MODULE_SETTINGS>(NVS_KEY_##pfx##_SETTINGS, &pfx##_CURRENT_SETTINGS, &pfx##_DEFAULT_SETTINGS)

#define RESET_EEPROM_SETINGS(pfx) \
        pfx##_CURRENT_SETTINGS = pfx##_DEFAULT_SETTINGS; \
        return EEPROM::write_subsystem_settings(NVS_KEY_##pfx##_SETTINGS, &pfx##_DEFAULT_SETTINGS); \

// Checks and writes the buffer as the setting.
// NOTE: The buffer points into the raw diagnostic payload, which carries no
// alignment guarantee. These structs contain floats and 32bit words, and an
// unaligned word access faults on Xtensa, so it must be memcpy'd out.
#define CHECK_AND_WRITE_SETTINGS(pfx, buffer_len, buffer) \
    if (sizeof(pfx##_MODULE_SETTINGS) != (buffer_len)) { \
        return ESP_ERR_INVALID_SIZE; \
    } else { \
        pfx##_MODULE_SETTINGS settings; \
        memcpy(&settings, (buffer), sizeof(pfx##_MODULE_SETTINGS)); \
        if (!sanitize_settings(&settings)) { \
            return ESP_ERR_INVALID_ARG; \
        } \
        pfx##_CURRENT_SETTINGS = settings; \
        return EEPROM::write_subsystem_settings(NVS_KEY_##pfx##_SETTINGS, &pfx##_CURRENT_SETTINGS); \
    } \

#define READ_SETTINGS_TO_BUFFER(pfx, buffer_len_dest, buffer_dest, use_default) \
    const pfx##_MODULE_SETTINGS* ptr = &pfx##_CURRENT_SETTINGS; \
    if ((use_default)) { \
        ptr = &pfx##_DEFAULT_SETTINGS; \
    } \
    uint8_t* dest = static_cast<uint8_t*>(TCU_HEAP_ALLOC(sizeof(pfx##_MODULE_SETTINGS)+1)); \
    if (nullptr == ptr || nullptr == dest) { \
        TCU_FREE(dest); \
        return ESP_ERR_NO_MEM; \
    } else { \
        dest[0] = pfx##_MODULE_SETTINGS_SCN_ID; \
        memcpy(&dest[1], ptr, sizeof(pfx##_MODULE_SETTINGS)); \
        *(buffer_len_dest) = sizeof(pfx##_MODULE_SETTINGS)+1; \
        *(buffer_dest) = dest; \
        return ESP_OK; \
    } \

// Records the first failure, but keeps loading the remaining modules so that a
// single bad NVS key does not leave the rest of the TCU unconfigured.
#define LOAD_EEPROM_SETTING(pfx, res) \
    { \
        esp_err_t load_res = READ_EEPROM_SETTING(pfx); \
        if (ESP_OK != load_res) { \
            ESP_LOGE("MODULE_SETTINGS", "Failed to load " #pfx " settings: %s", esp_err_to_name(load_res)); \
            if (ESP_OK == (res)) { \
                (res) = load_res; \
            } \
        } \
    }

esp_err_t ModuleConfiguration::load_all_settings() {
    esp_err_t res = ESP_OK;
    LOAD_EEPROM_SETTING(TCC, res); // Torque converter
    LOAD_EEPROM_SETTING(SOL, res); // Solenoid program
    LOAD_EEPROM_SETTING(SBS, res); // Shift basic control program
    LOAD_EEPROM_SETTING(PRM, res); // Pressure manager Settings
    LOAD_EEPROM_SETTING(ADP, res); // Adaptation settings
    LOAD_EEPROM_SETTING(ETS, res); // Electronic gear selector settings
    LOAD_EEPROM_SETTING(REL, res); // Release shift settings
    LOAD_EEPROM_SETTING(GAR, res); // Garage shift settings
    LOAD_EEPROM_SETTING(CRS, res); // Crossover shift settings
    // A stored block can predate a validation rule, so re-check what we loaded
    // and fall back to defaults rather than running with an unusable divisor.
    if (!sanitize_settings(&SOL_CURRENT_SETTINGS)) {
        ESP_LOGE("MODULE_SETTINGS", "Stored SOL settings are implausible, using defaults");
        SOL_CURRENT_SETTINGS = SOL_DEFAULT_SETTINGS;
        if (ESP_OK == res) {
            res = ESP_ERR_INVALID_STATE;
        }
    }
    return res;
}

esp_err_t ModuleConfiguration::reset_settings(uint8_t idx) {
    switch (idx) {
        case TCC_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(TCC)
            break;
        case SOL_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(SOL)
            break;
        case SBS_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(SBS)
            break;
        case PRM_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(PRM)
            break;
        case ADP_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(ADP)
            break;
        case ETS_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(ETS)
            break;
        case REL_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(REL)
            break;
        case GAR_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(GAR)
            break;
        case CRS_MODULE_SETTINGS_SCN_ID:
            RESET_EEPROM_SETINGS(CRS)
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t ModuleConfiguration::read_settings(uint8_t module_id, uint16_t* buffer_len, uint8_t** buffer) {
    if (buffer_len == nullptr || buffer == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t mod_id = module_id & 0b1111111;
    bool use_default = (module_id & BIT(7)) != 0;
    if (mod_id == TCC_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(TCC, buffer_len, buffer, use_default);
    } else if (mod_id == SOL_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(SOL, buffer_len, buffer, use_default);
    } else if (mod_id == SBS_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(SBS, buffer_len, buffer, use_default);
    } else if (mod_id == PRM_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(PRM, buffer_len, buffer, use_default);
    } else if (mod_id == ADP_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(ADP, buffer_len, buffer, use_default);
    } else if (mod_id == ETS_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(ETS, buffer_len, buffer, use_default);
    } else if (mod_id == REL_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(REL, buffer_len, buffer, use_default);
    } else if (mod_id == GAR_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(GAR, buffer_len, buffer, use_default);
    } else if (mod_id == CRS_MODULE_SETTINGS_SCN_ID) {
        READ_SETTINGS_TO_BUFFER(CRS, buffer_len, buffer, use_default);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t ModuleConfiguration::write_settings(uint8_t module_id, uint16_t buffer_len, uint8_t* buffer) {
    if (buffer == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (module_id == TCC_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(TCC, buffer_len, buffer)
    } else if (module_id == SOL_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(SOL, buffer_len, buffer)
    } else if (module_id == SBS_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(SBS, buffer_len, buffer)
    } else if (module_id == PRM_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(PRM, buffer_len, buffer)
    } else if (module_id == ADP_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(ADP, buffer_len, buffer)
    } else if (module_id == ETS_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(ETS, buffer_len, buffer)
    } else if (module_id == REL_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(REL, buffer_len, buffer)
    } else if (module_id == GAR_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(GAR, buffer_len, buffer)
    } else if (module_id == CRS_MODULE_SETTINGS_SCN_ID) {
        CHECK_AND_WRITE_SETTINGS(CRS, buffer_len, buffer)
    }
    return ESP_ERR_INVALID_ARG;
}