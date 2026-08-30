#ifndef KICKDOWNSWITCH_HPP
#define KICKDOWNSWITCH_HPP

#include "canbus/can_hal.h"

class KickdownSwitch {

public:
    static bool is_kickdown_newly_pressed(EgsBaseCan *egs_can_hal, uint32_t expire_time_ms);
    static bool is_kickdown_pressed(EgsBaseCan *egs_can_hal, uint32_t expire_time_ms);
private:
    static bool last_state; // Last state of the kickdown switch
};

#endif