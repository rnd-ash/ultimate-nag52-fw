#include "flasher_logic.h"

#include <limits.h>

bool flasher_range_fits_u32(uint32_t base, uint32_t size, uint32_t limit) {
    if (base > limit) {
        return false;
    }
    return size <= (limit - base);
}

bool flasher_try_align_up_u32(uint32_t value, uint32_t alignment, uint32_t* out) {
    if (out == nullptr || alignment == 0u) {
        return false;
    }

    uint32_t rem = value % alignment;
    if (rem == 0u) {
        *out = value;
        return true;
    }

    uint32_t delta = alignment - rem;
    if (value > UINT32_MAX - delta) {
        return false;
    }

    *out = value + delta;
    return true;
}