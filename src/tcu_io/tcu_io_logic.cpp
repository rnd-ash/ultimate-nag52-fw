#include "tcu_io_logic.h"

#include <limits.h>

uint16_t tcuio_calc_turbine_rpm_safe(uint16_t n2, uint16_t n3, float ratio_2_1) {
    float value = ((float)n2 * ratio_2_1) + ((float)n3 - (ratio_2_1 * (float)n3));
    if (!(value > 0.0f)) {
        return 0u;
    }
    if (value > (float)UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)value;
}