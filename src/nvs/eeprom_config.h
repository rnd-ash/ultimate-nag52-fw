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
#define NVS_KEY_TCC_ADAPTATION "TCC_ADAPTATION"

#define NVS_KEY_GEAR_ADAPTATION "GEAR_ADAPTATION"

typedef struct {
    bool is_large_nag;
    uint16_t diff_ratio;
    uint16_t wheel_diameter;
    bool is_four_matic;
    uint16_t transfer_case_high_ratio;
    uint16_t transfer_case_low_ratio;
} EEPROM_CORE_SCN_CONFIG;

struct TccAdaptationData { // 1 per gear
    // -40C - 120C (16 steps)
    bool learned[17];
    uint16_t slip_values[17]; // Where slip is (100-200RPM) (Torque is 50-120Nm)
};

namespace EEPROM {
    bool init_eeprom();
    uint8_t get_last_profile();
    bool read_core_scn_config(EEPROM_CORE_SCN_CONFIG* dest);
    bool save_core_scn_config(EEPROM_CORE_SCN_CONFIG* write);
    bool save_nvs_tcc_adaptation(TccAdaptationData* read_location, size_t store_size);
};

#define NUM_GEARS 5
extern TccAdaptationData torque_converter_adaptation[NUM_GEARS];

#endif