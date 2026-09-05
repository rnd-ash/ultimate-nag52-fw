#include "isotp_logic.h"

uint8_t isotp_single_frame_payload_len(uint8_t pci) {
    return (uint8_t)(pci & 0x0Fu);
}

bool isotp_first_frame_size_valid(uint16_t size, uint16_t max_size) {
    // First frames always carry 6 data bytes, so total payload must exceed that.
    return size > 6u && size <= max_size;
}

uint16_t isotp_clamp_payload_len(uint16_t payload_len, uint16_t max_size) {
    return (payload_len > max_size) ? max_size : payload_len;
}

uint8_t isotp_tx_chunk_size(uint16_t curr_pos, uint16_t max_pos) {
    if (curr_pos >= max_pos) {
        return 0u;
    }
    uint16_t remaining = max_pos - curr_pos;
    return (remaining > 7u) ? 7u : (uint8_t)remaining;
}

uint8_t isotp_rx_chunk_size(uint16_t curr_pos, uint16_t max_pos) {
    if (curr_pos >= max_pos) {
        return 0u;
    }
    uint16_t remaining = max_pos - curr_pos;
    return (remaining > 7u) ? 7u : (uint8_t)remaining;
}