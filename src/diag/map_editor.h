#ifndef __MAP_EDITOR_H_
#define __MAP_EDITOR_H_

#include <stdint.h>

#define MAP_ID_S_DIESEL_UPSHIFT 0x01
#define MAP_ID_S_DIESEL_DOWNSHIFT 0x02
#define MAP_ID_S_PETROL_UPSHIFT 0x03
#define MAP_ID_S_PETROL_DOWNSHIFT 0x04

#define MAP_ID_C_DIESEL_UPSHIFT 0x05
#define MAP_ID_C_DIESEL_DOWNSHIFT 0x06
#define MAP_ID_C_PETROL_UPSHIFT 0x07
#define MAP_ID_C_PETROL_DOWNSHIFT 0x08

#define MAP_ID_A_DIESEL_UPSHIFT 0x09
#define MAP_ID_A_DIESEL_DOWNSHIFT 0x0A
#define MAP_ID_A_PETROL_UPSHIFT 0x0B
#define MAP_ID_A_PETROL_DOWNSHIFT 0x0C



namespace MapEditor {
    uint8_t read_map_data(uint8_t map_id, uint16_t *dest_size, int16_t** buffer);
    uint8_t write_map_data(uint8_t map_id, uint16_t dest_size, int16_t* buffer);
    uint8_t trigger_reload(uint8_t prof_id);
}

#endif