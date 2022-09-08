#ifndef __MAP_EDITOR_H_
#define __MAP_EDITOR_H_

#include <stdint.h>

// Read/write data by identifier 3rd param IDs for which map to R/W
// (NOT USING LOCAL IDENTIFIER - KWP2000 spec LOCAL IDENTIFIER SHOULD NOT BE USED FOR THIS SORT OF FUNCTIONALITY)
#define DI_MAP_DATA 0xFD00 // Data identifier ID for map data


/** Current map IDs (Stored on NVS). Default maps are stored at ID + 0x20 **/

#define MAP_ID_C_UPSHIFT_DIESEL 0x01
#define MAP_ID_C_DOWNSHIFT_DIESEL 0x02
#define MAP_ID_C_UPSHIFT_PETROL 0x03
#define MAP_ID_C_DOWNSHIFT_PETROL 0x04

#define MAP_ID_S_UPSHIFT_DIESEL 0x05
#define MAP_ID_S_DOWNSHIFT_DIESEL 0x06
#define MAP_ID_S_UPSHIFT_PETROL 0x07
#define MAP_ID_S_DOWNSHIFT_PETROL 0x08

#define MAP_ID_A_UPSHIFT_DIESEL 0x09
#define MAP_ID_A_DOWNSHIFT_DIESEL 0x0A
#define MAP_ID_A_UPSHIFT_PETROL 0x0B
#define MAP_ID_A_DOWNSHIFT_PETROL 0x0C

namespace MapEditor {
    uint8_t read_map();
    uint8_t write_map();
    uint8_t reload_
};

#endif