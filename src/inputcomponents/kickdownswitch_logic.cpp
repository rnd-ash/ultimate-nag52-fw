#include "kickdownswitch_logic.h"

bool kickdown_is_new_press(bool current_state, bool last_state) {
    return current_state && !last_state;
}