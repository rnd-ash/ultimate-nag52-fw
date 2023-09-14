#ifndef SHIFTER_TRRS_H
#define SHIFTER_TRRS_H

#include "shifter.h"
#include "esp_err.h"

class ShifterTrrs : public Shifter
{
public:
    explicit ShifterTrrs(esp_err_t *can_init_status);
    ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
    ProfileSwitchPos get_shifter_profile_switch_pos(const uint32_t expire_time_ms) override;
    void set_rp_solenoid(const float vVeh, const ShifterPosition pos, const bool is_brake_pressed);
private:
    const ShifterPosition TRRS_SHIFTER_TABLE[16] = {
        ShifterPosition::SignalNotAvailable,    // 0b0000
        ShifterPosition::TWO,                   // 0b0001
        ShifterPosition::THREE,                 // 0b0010
        ShifterPosition::SignalNotAvailable,    // 0b0011
        ShifterPosition::D,                     // 0b0100
        ShifterPosition::SignalNotAvailable,    // 0b0101
        ShifterPosition::SignalNotAvailable,    // 0b0110
        ShifterPosition::P,                     // 0b0111
        ShifterPosition::FOUR,                  // 0b1000
        ShifterPosition::SignalNotAvailable,    // 0b1001
        ShifterPosition::SignalNotAvailable,    // 0b1010
        ShifterPosition::ONE,                   // 0b1011
        ShifterPosition::SignalNotAvailable,    // 0b1100
        ShifterPosition::N,                     // 0b1101
        ShifterPosition::R,                     // 0b1110
        ShifterPosition::SignalNotAvailable     // 0b1111
    };

    ShifterPosition last_valid_position = ShifterPosition::SignalNotAvailable;
};

#endif // SHIFTER_TRRS_H