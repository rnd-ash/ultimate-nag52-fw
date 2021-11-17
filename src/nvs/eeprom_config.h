#ifndef __EEPROM_CONFIG_H_
#define __EEPROM_CONFIG_H_

#include "esp_err.h"
#include "esp_log.h"

#define NVS_KEY_EEPROM_INIT "EEPROM_INIT"
#define NVS_KEY_DRIVE_PROG "DRIVE_PROFILE"
#define NVS_KEY_DIFF_RATIO "DIFF_RATIO"
#define NVS_KEY_TCC_LOCKUP "TCC_LOCKUP"
#define NVS_KEY_WHEEL_DIAM "WHEEL_DIAM"

typedef struct {

} EEPROM_Config;

namespace EEPROM {
    bool init_eeprom();
};

#endif