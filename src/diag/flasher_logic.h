#ifndef FLASHER_LOGIC_H
#define FLASHER_LOGIC_H

#include <stdint.h>

bool flasher_range_fits_u32(uint32_t base, uint32_t size, uint32_t limit);
bool flasher_try_align_up_u32(uint32_t value, uint32_t alignment, uint32_t* out);

#endif