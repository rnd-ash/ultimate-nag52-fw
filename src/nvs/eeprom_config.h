#ifndef __EEPROM_CONFIG_H_
#define __EEPROM_CONFIG_H_

#include "esp_err.h"
#include "esp_log.h"

#define NVS_KEY_EEPROM_INIT "EEPROM_INIT"
#define NVS_KEY_DRIVE_PROG "DRIVE_PROFILE"
#define NVS_KEY_DIFF_RATIO "DIFF_RATIO"
#define NVS_KEY_TCC_LOCKUP "TCC_LOCKUP"
#define NVS_KEY_WHEEL_DIAM "WHEEL_DIAM"

#define NVS_PARTITION_USER_CFG "tcm_user_config"
#define NVS_UCFG_KEY_PROFILE "LAST_PROFILE"
#define NVS_KEY_TCC_ADAPTATION "TCC_ADAPTATION"

typedef struct {

} EEPROM_Config;

struct TccAdaptationData { // 1 per gear
    // -40C - 120C (16 steps)
    uint16_t lockup_values[17]; // Where slip is < 100RPM (Torque is 50-120Nm)
    uint16_t slip_values[17]; // Where slip is (100-200RPM) (Torque is 50-120Nm)
    uint16_t slip_rpm_limit; // Max RPM Delta for slipping
    uint16_t lock_rpm_limit; // Max RPM Delta for locked
};

namespace EEPROM {
    bool init_eeprom();
    uint8_t get_last_profile();
    bool save_nvs_tcc_adaptation(TccAdaptationData* read_location, size_t store_size);
};

#define NUM_GEARS 5
extern TccAdaptationData torque_converter_adaptation[NUM_GEARS];

#endif