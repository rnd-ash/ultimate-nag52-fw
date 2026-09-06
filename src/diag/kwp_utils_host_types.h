#ifndef KWP_UTILS_HOST_TYPES_H
#define KWP_UTILS_HOST_TYPES_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus

static constexpr size_t DIAG_CAN_MAX_SIZE = 4095u;

// cppcheck-suppress syntaxError
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

#else

#define DIAG_CAN_MAX_SIZE 4095u

typedef enum {
    ShifterPosition_P = 0,
    ShifterPosition_R,
    ShifterPosition_N,
    ShifterPosition_D,
    ShifterPosition_PLUS,
    ShifterPosition_MINUS,
    ShifterPosition_N_D,
    ShifterPosition_P_R,
    ShifterPosition_R_N,
    ShifterPosition_SignalNotAvailable
} ShifterPosition;

typedef struct {
    uint16_t id;
    uint16_t data_size;
    uint8_t data[DIAG_CAN_MAX_SIZE];
} DiagMessage;

typedef struct EgsBaseCan EgsBaseCan;

#endif

#endif