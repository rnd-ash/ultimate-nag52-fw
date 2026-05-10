#include "kickdownswitch.hpp"
#include "board_config.h"


bool KickdownSwitch::last_state = false; // Initialize the last state to false

bool KickdownSwitch::is_kickdown_newly_pressed(EgsBaseCan *egs_can_hal, uint32_t expire_time_ms)
{
    bool current_state = is_kickdown_pressed(egs_can_hal, expire_time_ms);
    bool result = current_state != KickdownSwitch::last_state;
    KickdownSwitch::last_state = current_state;
    return result;
}

bool KickdownSwitch::is_kickdown_pressed(EgsBaseCan *egs_can_hal, uint32_t expire_time_ms)
{
    return pcb_gpio_matrix->is_kickdown_pressed() || egs_can_hal->get_kickdown(expire_time_ms);
}
