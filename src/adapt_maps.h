#ifndef ADAPT_MAP_H_
#define ADAPT_MAP_H_
/*
    This file contains mapping for adaptation maps for various subsystems on the TCU
*/

#include <cstdint>
#include <stdint.h>

#define TCC_ADAPT_MAP_Z_SIZE 6*6
extern const int16_t TCC_ADAPT_MAP_X[6];
extern const int16_t TCC_ADAPT_MAP_Y[6];

extern const int16_t TCC_ADAPT_MAP_Z[TCC_ADAPT_MAP_Z_SIZE];

#endif
