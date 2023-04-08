#ifndef SHIFTER_H
#define SHIFTER_H

#include <stdint.h>

enum ShifterStyle : uint8_t {
	EWM = 0u,
	TRRS = 1u,
	SLR = 2u
};

enum ShifterPosition : uint8_t {
    P,
    P_R,
    R,
    R_N,
    N,
    N_D,
    D,
    PLUS, // For EWM only
    MINUS, // For EWM only
    FOUR, // For TRRS only
    THREE, // For TRRS only
    TWO, // For TRRS only
    ONE, // For TRRS only
    SignalNotAvailable = UINT8_MAX // SNV
};

class Shifter
{
public:
	virtual ShifterPosition get_shifter_position(const uint64_t now, const uint64_t expire_time_ms) = 0;
};

#endif // SHIFTER_H