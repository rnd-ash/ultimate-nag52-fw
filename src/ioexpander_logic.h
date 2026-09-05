#ifndef IOEXPANDER_LOGIC_H
#define IOEXPANDER_LOGIC_H

#include <stdint.h>

bool ioexpander_is_valid_bit(int bit);
bool ioexpander_get_input_bit(uint8_t input_byte, int bit);
uint8_t ioexpander_set_output_bit(uint8_t output_byte, int bit, bool value);

#endif