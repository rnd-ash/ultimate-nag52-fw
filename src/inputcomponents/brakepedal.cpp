#include "brakepedal.hpp"
#include "board_config.h"

bool BrakePedal::is_brake_pedal_pressed(EgsBaseCan * egs_can_hal, uint32_t expire_time_ms) {
    bool pcb_pressed = pcb_gpio_matrix != nullptr && pcb_gpio_matrix->is_brake_light_switch_pressed();
    bool can_pressed = egs_can_hal != nullptr && egs_can_hal->get_is_brake_pressed(expire_time_ms);
    return pcb_pressed || can_pressed;
}