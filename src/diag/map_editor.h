#ifndef __MAP_EDITOR_H_
#define __MAP_EDITOR_H_

#include <stdint.h>
#include "kwp2000_defines.h"

/*-
 * #name: Agility upshift threshold
 * #desc: Agility mode upshift threshold map
 * #celldesc: Input shaft speed to change at
 * #ydesc: Gear change
 * #xdesc: Pedal input
 * #xunit: %
 * #cellunit: RPM
 * #yov: ["1-2", "2-3", "3-4", "4-5"]
 -*/
#define A_UPSHIFT_MAP_ID 0x01
/*-
 * #name: Comfort upshift threshold
 * #desc: Comfort mode upshift threshold map
 * #celldesc: Input shaft speed to change at
 * #ydesc: Gear change
 * #xdesc: Pedal input
 * #xunit: %
 * #cellunit: RPM
 * #yov: ["1-2", "2-3", "3-4", "4-5"]
 -*/
#define C_UPSHIFT_MAP_ID 0x02
#define S_UPSHIFT_MAP_ID 0x03
#define A_DOWNSHIFT_MAP_ID 0x04
#define C_DOWNSHIFT_MAP_ID 0x05
#define S_DOWNSHIFT_MAP_ID 0x06

/*-
 * #name: Working pressure
 * #desc: Working gearbox pressure based on load
 * #celldesc: Desired pressure
 * #cellunit: mBar
 * #ydesc: Gear
 * #xdesc: Input shaft torque % of rated
 * #xunit: %
 * #yov: ["P/R", "R1/R2", "1", "2", "3", "4", "5"]
 -*/
#define WORKING_PRESSURE_MAP_ID 0x07
/*-
 * #name: Pressure control solenoid current
 * #desc: Pressure control solenoid current mapped to gearbox pressure
 * #celldesc: Current
 * #cellunit: mA
 * #ydesc: ATF Temp
 * #yunit: *C
 * #xdesc: Desired pressure
 * #xunit: mBar
 -*/
#define PCS_CURRENT_MAP_ID 0x08
/*-
 * #name: TCC Solenoid duty
 * #desc: Pressure mapping between TCC solenoid PWM and TCC clutch pressure
 * #celldesc: Solenoid duty
 * #cellunit: /4096
 * #ydesc: ATF Temp
 * #yunit: *C
 * #xdesc: Desired TCC pressure
 * #xunit: mBar
 -*/
#define TCC_PWM_MAP_ID 0x09
/*-
 * #name: Clutch filling time
 * #desc: Clutch filling time for shift fill phase
 * #celldesc: Fill time
 * #cellunit: ms
 * #xdesc: Clutch
 * #xov: ["K1", "K2", "K3", "B1", "B2"]
 -*/
#define FILL_TIME_MAP_ID 0x0A

/*-
 * #name: Clutch filling pressure
 * #desc: Clutch filling pressure for shift fill phase
 * #celldesc: Fill pressure
 * #cellunit: mBar
 * #xdesc: Clutch
 * #xov: ["K1", "K2", "K3", "B1", "B2"]
 -*/
#define FILL_PRESSURE_MAP_ID 0x0B

#define A_UPTIME_MAP_ID 0x10
#define A_DNTIME_MAP_ID 0x11
#define S_UPTIME_MAP_ID 0x12
#define S_DNTIME_MAP_ID 0x13
#define C_UPTIME_MAP_ID 0x14
#define C_DNTIME_MAP_ID 0x15
#define W_UPTIME_MAP_ID 0x16
#define W_DNTIME_MAP_ID 0x17
#define M_UPTIME_MAP_ID 0x18
#define M_DNTIME_MAP_ID 0x19


// MAP COMMAND IDs
#define MAP_CMD_READ 0x01
#define MAP_CMD_READ_DEFAULT 0x02
#define MAP_CMD_WRITE 0x03
#define MAP_CMD_BURN 0x04
#define MAP_CMD_RESET_TO_FLASH 0x05
#define MAP_CMD_UNDO 0x06
#define MAP_CMD_READ_META 0x07
#define MAP_CMD_READ_EEPROM 0x08

#define MAP_READ_TYPE_MEM 0x01
#define MAP_READ_TYPE_PRG 0x02
#define MAP_READ_TYPE_STO 0x03

namespace MapEditor {
    kwp_result_t read_map_metadata(uint8_t map_id, uint16_t *dest_size_bytes, uint8_t** buffer);
    kwp_result_t read_map_data(uint8_t map_id, uint8_t read_type, uint16_t *dest_size_bytes, uint8_t** buffer);
    kwp_result_t write_map_data(uint8_t map_id, uint16_t dest_size, int16_t* buffer);
    kwp_result_t burn_to_eeprom(uint8_t map_id);
    kwp_result_t reset_to_program_default(uint8_t map_id);
    kwp_result_t undo_changes(uint8_t map_id);
}

#endif