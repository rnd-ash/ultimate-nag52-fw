#ifndef TCC_ADAPTATION_H__
#define TCC_ADAPTATION_H__

#include <stdint.h>

inline void update_tcc_adaptation_row(
    int16_t* row,
    uint8_t row_size,
    uint8_t load_idx,
    int16_t offset
) {
    int32_t adjusted = (int32_t)row[load_idx] + offset;
    if (adjusted < 100) {
        adjusted = 100;
    } else if (adjusted > 15000) {
        adjusted = 15000;
    }
    if (load_idx > 0 && adjusted < row[load_idx - 1]) {
        adjusted = row[load_idx - 1];
    }
    row[load_idx] = (int16_t)adjusted;

    // TCC holding pressure must not decrease as gearbox load increases.
    // Only normalize from the adapted cell onward so unrelated calibration
    // cells are not modified.
    for (uint8_t i = load_idx + 1; i < row_size; i++) {
        if (row[i] < row[i - 1]) {
            row[i] = row[i - 1];
        }
    }
}

#endif
