/** @file */
#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H

#include "esp_err.h"
#include "esp_log.h"
#include "common_structs.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include <string.h>

static const char NVS_PARTITION_USER_CFG[16] = "tcm_user_config";

struct __attribute__ ((packed)) TCM_CORE_CONFIG{
    uint8_t deprecated_is_large_nag;
    uint16_t diff_ratio;
    uint16_t wheel_circumference;
    uint8_t is_four_matic;
    uint16_t transfer_case_high_ratio;
    uint16_t transfer_case_low_ratio;
    uint8_t default_profile;
    uint16_t red_line_rpm_diesel;
    uint16_t red_line_rpm_petrol;
    uint8_t engine_type; // 0 for diesel, 1 for petrol
    uint8_t egs_can_type;
    // 0 - EWM
    // 1 - TRRS
    // 2 - SLR
    uint8_t shifter_style;
    // 0 - N/C
    // 1 - Vehicle speed sensor
    // 2 - Output (TBA)
    // 3 - TCC Manual MOSFET (1.3 ONLY)
    uint8_t io_0_usage;
    uint8_t input_sensor_pulses_per_rev;
    uint8_t output_pulse_width_per_kmh;
    uint8_t gen_mosfet_purpose;
    // values required for classic Hfm
    // maximum opening angle of the throttle valve (in [°] to be more universal)
    uint8_t throttlevalve_maxopeningangle;
    // constant factor required to convert mdot and engine speed to the engine torque (in [m²/s²] * 1000)
    uint16_t c_eng;
    // Engine drag torque (Nm/10)
    uint16_t engine_drag_torque;
    // Boolean for Jeep/Chrysler mode
    bool jeep_chrysler;
};


// -- EFuse layout --
//   Key                Block     start bit    length bit
// PRODUCT.BOARD_VER, EFUSE_BLK3, 0,           8
// PRODUCT.M_DAY,     EFUSE_BLK3, 8,           8
// PRODUCT.M_WEEK,    EFUSE_BLK3, 16,          8
// PRODUCT.M_MONTH,   EFUSE_BLK3, 24,          8
// PRODUCT.M_YEAR,    EFUSE_BLK3, 32,          8

struct __attribute__ ((packed)) TCM_EFUSE_CONFIG {
    // 1 - Red PCB (V1.1)
    // 2 - Blue PCB (V1.2)
    // 3 - Black PCB (V1.3)
    // 4 - Green PCB (V1.4 - PRE PROD)
    // 0xF | 4 - Green PCB HGS Variant
    uint8_t board_ver;
    uint8_t manufacture_day;
    uint8_t manufacture_week;
    uint8_t manufacture_month;
    uint8_t manufacture_year;
};

struct __attribute__ ((packed)) FLASH_NVS_SETTINGS_DESC {
    uint8_t magic[4];
    uint32_t key_cs;
    uint32_t compressed_size;
};

namespace EEPROM {
    esp_err_t init_eeprom(void);
    esp_err_t ewm_btn_get_saved_profile(uint8_t* dest);
    esp_err_t ewm_btn_save_profile(uint8_t save_profile);
    esp_err_t read_core_config(TCM_CORE_CONFIG* dest);
    esp_err_t save_core_config(TCM_CORE_CONFIG* write);
    esp_err_t read_efuse_config(TCM_EFUSE_CONFIG* dest);
    esp_err_t write_efuse_config(TCM_EFUSE_CONFIG* dest);

    uint16_t read_device_mode(void);
    esp_err_t set_device_mode(uint16_t mode);

    esp_err_t read_nvs_map_data(const char* map_name, int16_t* dest, const int16_t* default_map, size_t map_element_count);
    esp_err_t write_nvs_map_data(const char* map_name, const int16_t* to_write, size_t map_element_count);
    
    esp_err_t check_if_new_fw(bool* dest);

    template <typename T>
    esp_err_t read_subsystem_settings(const char* key_name, T* dest, const T* default_settings);

    template <typename T>
    esp_err_t write_subsystem_settings(const char* key_name, const T* write);
}

#define NUM_GEARS 5
extern TCM_CORE_CONFIG VEHICLE_CONFIG;
extern TCM_EFUSE_CONFIG BOARD_CONFIG;
extern nvs_handle_t MAP_NVS_HANDLE;

#endif