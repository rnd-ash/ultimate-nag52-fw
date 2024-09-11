
#ifndef KWP_UTILS_H
#define KWP_UTILS_H

#include "endpoints/endpoint.h"
#include "stdint.h"

// Couple of helpful functions
void global_make_diag_neg_msg(DiagMessage *dest, uint8_t sid, uint8_t nrc);

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, const uint8_t* resp, uint16_t len);

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, uint8_t pid, const uint8_t* resp, uint16_t len);

bool is_engine_off(const EgsBaseCan* can);
bool is_shifter_passive(EgsBaseCan* can);

/* unused */
// const char* kwp_nrc_to_name(kwp_result_t res);

#endif