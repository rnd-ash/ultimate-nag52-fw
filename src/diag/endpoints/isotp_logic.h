#ifndef ISOTP_LOGIC_H
#define ISOTP_LOGIC_H

#include <stdint.h>

uint8_t isotp_single_frame_payload_len(uint8_t pci);
bool isotp_first_frame_size_valid(uint16_t size, uint16_t max_size);
uint16_t isotp_clamp_payload_len(uint16_t payload_len, uint16_t max_size);
uint8_t isotp_tx_chunk_size(uint16_t curr_pos, uint16_t max_pos);
uint8_t isotp_rx_chunk_size(uint16_t curr_pos, uint16_t max_pos);

#endif