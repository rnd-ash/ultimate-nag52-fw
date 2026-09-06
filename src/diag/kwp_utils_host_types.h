#ifndef KWP_UTILS_HOST_TYPES_H
#define KWP_UTILS_HOST_TYPES_H

#include <stdint.h>

static const uint16_t DIAG_CAN_MAX_SIZE = 4095u;

enum class ShifterPosition : uint8_t {
    P = 0,
    R,
    N,
    D,
    PLUS,
    MINUS,
    N_D,
    P_R,
    R_N,
    SignalNotAvailable
};

struct DiagMessage {
    uint16_t id;
    uint16_t data_size;
    uint8_t data[DIAG_CAN_MAX_SIZE];
};

class EgsBaseCan {
public:
    virtual ~EgsBaseCan() = default;
    virtual uint16_t get_engine_rpm(uint32_t expire_time_ms) = 0;
    virtual ShifterPosition get_shifter_position(uint32_t expire_time_ms) = 0;
};

#endif