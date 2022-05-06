
#ifndef __KWP_UTILS_H__
#define __KWP_UTILS_H__

#include "endpoint.h"
#include "stdint.h"

// Couple of helpful functions
void global_make_diag_neg_msg(DiagMessage *dest, uint8_t sid, uint8_t nrc);

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, const uint8_t* resp, uint16_t len);

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, uint8_t pid, const uint8_t* resp, uint16_t len);

bool is_engine_off(AbstractCan* can);
bool is_shifter_passive(AbstractCan* can);

#endif