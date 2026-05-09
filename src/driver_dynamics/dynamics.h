#ifndef __DRIVER_DYNAMICS_H_
#define __DRIVER_DYNAMICS_H_

#include <stdint.h>

class DeltaTracker {
public:
    DeltaTracker(uint8_t samples);
    void update(int32_t val);
    void reset();
    int32_t get_delta();
private:
    uint8_t samples = 0;
    int32_t last_value = 0;
    bool first_val = true;
    float tracked_delta = 0;
};

#endif