#ifndef SHIFTER_H
#define SHIFTER_H

#include <stdint.h>

enum ShifterStyle{
	EWM = 0u,
	TRRS = 1u,
	SLR = 2u
};

enum ShifterPosition{
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
protected:
	uint64_t *_last_i2c_query_time;
	uint8_t *_i2c_rx_bytes;
};

#endif // SHIFTER_H