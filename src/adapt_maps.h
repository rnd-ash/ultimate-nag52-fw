#ifndef ADAPT_MAP_H_
#define ADAPT_MAP_H_
/*
    This file contains mapping for adaptation maps for various subsystems on the TCU
*/

#include <stdint.h>

#define TCC_SLIP_ADAPT_MAP_SIZE 5*11

extern const int16_t TCC_SLIP_ADAPT_MAP[TCC_SLIP_ADAPT_MAP_SIZE];
extern const int16_t TCC_LOCK_ADAPT_MAP[TCC_SLIP_ADAPT_MAP_SIZE];

#endif