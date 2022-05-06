/** @file */
#ifndef __EEPROM_CONFIG_H_
#define __EEPROM_CONFIG_H_

#include "esp_err.h"
#include "esp_log.h"
#include "common_structs.h"

#define NVS_KEY_EEPROM_INIT "EEPROM_INIT"

// Core SCN config (Needed for a ton of important calculations!)
#define NVS_KEY_SCN_CONFIG "CORE_SCN"


#define NVS_PARTITION_USER_CFG "tcm_user_config"
#define NVS_UCFG_KEY_PROFILE "LAST_PROFILE"
#define NVS_KEY_GEAR_ADAPTATION "GEAR_ADAPTATION"

typedef struct {
    uint8_t is_large_nag;
    uint16_t diff_ratio;
    uint16_t wheel_circumference;
    uint8_t is_four_matic;
    uint16_t transfer_case_high_ratio;
    uint16_t transfer_case_low_ratio;
    uint8_t default_profile;
    uint16_t red_line_rpm_diesel;
    uint16_t red_line_rpm_petrol;
    uint8_t engine_type; // 0 for diesel, 1 for petrol
} __attribute__ ((packed)) TCM_CORE_CONFIG;

namespace EEPROM {
    bool init_eeprom();
    uint8_t get_last_profile();
    bool read_core_config(TCM_CORE_CONFIG* dest);
    bool save_core_config(TCM_CORE_CONFIG* write);
};

#define NUM_GEARS 5
extern TCM_CORE_CONFIG VEHICLE_CONFIG;

#endif