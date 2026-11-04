#pragma once
#include <stdint.h>

namespace Egs51LineCoupling {
// Bank1 D76E: only shift index 1 subtracts SPC * FFEB / 100 from
// estimated working pressure, before inlet correction. Do not wrap at zero.
inline uint32_t working_pressure(uint32_t line, bool first_upshift,
                                 uint16_t spc, uint16_t percent) {
    if (!first_upshift) return line;
    const uint64_t reduction = uint64_t(spc) * percent / 100;
    return reduction >= line ? 0 : uint32_t(line - reduction);
}
}
