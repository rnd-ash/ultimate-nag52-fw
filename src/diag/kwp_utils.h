
#ifndef __KWP_UTILS_H__
#define __KWP_UTILS_H__

#ifdef KWP_UTILS_HOST_STUB
#include "kwp_utils_host_types.h"
#else
#include "endpoints/endpoint.h"
#endif
#include "stdint.h"

// Couple of helpful functions
void global_make_diag_neg_msg(DiagMessage *dest, uint8_t sid, uint8_t nrc);

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, const uint8_t* resp, uint16_t len);

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, uint8_t pid, const uint8_t* resp, uint16_t len);

bool is_engine_off(EgsBaseCan* can);
bool is_shifter_passive(EgsBaseCan* can);

#endif