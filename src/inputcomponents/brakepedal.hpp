#ifndef BRAKEPEDAL_HPP
#define BRAKEPEDAL_HPP

#include "canbus/can_hal.h"

class BrakePedal {

public:
    /**
     * @brief Check if the kickdown switch is pressed
     * @return true if pressed, false otherwise
     */
    static bool is_brake_pedal_pressed(EgsBaseCan *egs_can_hal, const uint32_t expire_time_ms);  
};
#endif