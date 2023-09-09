#ifndef SHIFTER_H
#define SHIFTER_H

#include <stdint.h>
#include "clock.hpp"

enum class ShifterStyle : uint8_t {
	EWM = 0u,
	TRRS = 1u,
	SLR = 2u,
};

enum class ProfileSwitchPos : uint8_t {
    Top = 0u,
    Bottom = 1u,
    SNV = UINT8_MAX,
};

enum class ShifterPosition : uint8_t {
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
    SignalNotAvailable = UINT8_MAX, // SNV
};

class Shifter
{
public:
	virtual ShifterPosition get_shifter_position(const uint32_t expire_time_ms) = 0;
    virtual ProfileSwitchPos get_shifter_profile_switch_pos(const uint32_t expire_time_ms) = 0;
};

#endif // SHIFTER_H