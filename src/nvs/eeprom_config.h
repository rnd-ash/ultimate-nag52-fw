#ifndef __EEPROM_CONFIG_H_
#define __EEPROM_CONFIG_H_

#include "esp_err.h"
#include "esp_log.h"

#define NVS_KEY_EEPROM_INIT "EEPROM_INIT"
#define NVS_KEY_DRIVE_PROG "DRIVE_PROFILE"
#define NVS_KEY_DIFF_RATIO "DIFF_RATIO"
#define NVS_KEY_TCC_LOCKUP "TCC_LOCKUP"
#define NVS_KEY_WHEEL_DIAM "WHEEL_DIAM"

#define NVS_KEY_1_2 "1TO2"
#define NVS_KEY_2_3 "2TO3"
#define NVS_KEY_3_4 "3TO4"
#define NVS_KEY_4_5 "4TO5"

#define NVS_KEY_2_1 "2TO1"
#define NVS_KEY_3_2 "3TO2"
#define NVS_KEY_4_3 "4TO3"
#define NVS_KEY_5_4 "5TO4"

typedef struct {

} EEPROM_Config;

namespace EEPROM {
    bool init_eeprom();
    bool save_map_data();
    bool load_map_data();
};

#define RPM_STEP 250

#define MAX_RPM 5000/RPM_STEP // 20
#define MIN_RPM 1000/RPM_STEP // 4
#define PEDAL_STEPS 250/25 // 10
#define ATF_TEMP_STAGES 12 // 12 (Start at 0)

// TOTAL Bytes per gear change = (20-4)*10*12*2 = 3840bytes
// 10 gear changes = 39KB

typedef uint16_t SHIFT_LAYER[ATF_TEMP_STAGES][MAX_RPM-MIN_RPM][PEDAL_STEPS];

extern SHIFT_LAYER* one_to_two;
extern SHIFT_LAYER* two_to_three;
extern SHIFT_LAYER* three_to_four;
extern SHIFT_LAYER* four_to_five;

extern SHIFT_LAYER* five_downto_four;
extern SHIFT_LAYER* four_downto_three;
extern SHIFT_LAYER* three_downto_two;
extern SHIFT_LAYER* two_downto_one;

#endif