#include "brakepedal.hpp"
#include "board_config.h"

bool BrakePedal::is_brake_pedal_pressed(EgsBaseCan * egs_can_hal, uint32_t expire_time_ms) {
    return pcb_gpio_matrix->is_brake_light_switch_pressed() || egs_can_hal->get_is_brake_pressed(expire_time_ms);
}