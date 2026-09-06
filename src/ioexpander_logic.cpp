#include "ioexpander_logic.h"

bool ioexpander_is_valid_bit(int bit) {
    return bit >= 0 && bit <= 7;
}

bool ioexpander_get_input_bit(uint8_t input_byte, int bit) {
    if (!ioexpander_is_valid_bit(bit)) {
        return false;
    }
    return ((input_byte >> bit) & 0x01u) != 0u;
}

uint8_t ioexpander_set_output_bit(uint8_t output_byte, int bit, bool value) {
    if (!ioexpander_is_valid_bit(bit)) {
        return output_byte;
    }
    uint8_t mask = (uint8_t)(1u << bit);
    output_byte &= (uint8_t)(~mask);
    if (value) {
        output_byte |= mask;
    }
    return output_byte;
}