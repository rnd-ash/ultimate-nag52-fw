#ifndef SHIFTER_TRRS_H
#define SHIFTER_TRRS_H

#include "shifter.h"
#include "esp_err.h"
#include "../board_config.h"
#include "programselector/programselectorswitchtrrs.h"

class ShifterTrrs : public Shifter
{
public:
    ShifterTrrs(TCM_CORE_CONFIG* vehicle_config, BoardGpioMatrix *board);
    ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;    
    AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
    DiagProfileInputState diag_get_profile_input() override;
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

    void set_rp_solenoid(const float vVeh, const ShifterPosition pos, const bool is_brake_pressed);

    ShifterPosition last_valid_position = ShifterPosition::SignalNotAvailable;
    BoardGpioMatrix* board;
    ProgramSelectorSwitchTRRS* programselector;
};

#endif // SHIFTER_TRRS_H