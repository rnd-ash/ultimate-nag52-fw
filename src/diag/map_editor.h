#ifndef __MAP_EDITOR_H_
#define __MAP_EDITOR_H_

#include <stdint.h>

#define A_UPSHIFT_MAP_ID 0x01
#define C_UPSHIFT_MAP_ID 0x02
#define S_UPSHIFT_MAP_ID 0x03
#define A_DOWNSHIFT_MAP_ID 0x04
#define C_DOWNSHIFT_MAP_ID 0x05
#define S_DOWNSHIFT_MAP_ID 0x06

#define WORKING_PRESSURE_MAP_ID 0x07
#define PCS_CURRENT_MAP_ID 0x08
#define TCC_PWM_MAP_ID 0x09
#define FILL_TIME_MAP_ID 0x0A
#define FILL_PRESSURE_MAP_ID 0x0B


// MAP COMMAND IDs
#define MAP_CMD_READ 0x01
#define MAP_CMD_READ_DEFAULT 0x02
#define MAP_CMD_WRITE 0x03
#define MAP_CMD_BURN 0x04
#define MAP_CMD_RESET_TO_FLASH 0x05
#define MAP_CMD_UNDO 0x06


namespace MapEditor {
    uint8_t read_map_data(uint8_t map_id, bool is_default, uint16_t *dest_size, int16_t** buffer);
    uint8_t write_map_data(uint8_t map_id, uint16_t dest_size, int16_t* buffer);
    uint8_t burn_to_eeprom(uint8_t map_id);
    uint8_t reset_to_program_default(uint8_t map_id);
    uint8_t undo_changes(uint8_t map_id);
}

#endif