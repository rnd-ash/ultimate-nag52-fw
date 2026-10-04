#include "adapt_maps.h"
#include <cstdint>


const int16_t TCC_ADAPT_MAP_X[6] = {0, 5, 10, 25, 50, 100};
const int16_t TCC_ADAPT_MAP_Y[6] = {-20, 0, 20, 40, 80, 120};

const int16_t TCC_ADAPT_MAP_Z[TCC_ADAPT_MAP_Z_SIZE] = { // Z - Hydraulic Pressure
    //   0, 5, 10, 25, 50, 100  Torque (%)
         0, 0, 0, 0, 0, 0, // -20 C
    0, 0, 0, 0, 0, 0,// 0 C
    0, 0, 0, 0, 0, 0,// 20 C
    0, 0, 0, 0, 0, 0,// 40 C
    0, 0, 0, 0, 0, 0,// 80 C
    0, 0, 0, 0, 0, 0,// 120C
};
