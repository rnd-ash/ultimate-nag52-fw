#include "common_structs_ops.h"

ShiftStage next_shift_stage(ShiftStage now) {
    switch(now) {
        case ShiftStage::Bleed:
            return ShiftStage::Fill;
        case ShiftStage::Fill:
            return ShiftStage::Overlap;
        case ShiftStage::Overlap:
        case ShiftStage::MaxPressure:
        default:
            return ShiftStage::MaxPressure;
    }
}

const Clutch get_clutch_to_apply(ProfileGearChange change) {
    switch(change) {
        case ProfileGearChange::ONE_TWO:
        case ProfileGearChange::FIVE_FOUR:
            return Clutch::K1;
        case ProfileGearChange::TWO_THREE:
            return Clutch::K2;
        case ProfileGearChange::THREE_FOUR:
        case ProfileGearChange::THREE_TWO:
            return Clutch::K3;
        case ProfileGearChange::FOUR_THREE:
            return Clutch::B2;
        case ProfileGearChange::FOUR_FIVE:
        case ProfileGearChange::TWO_ONE:
        default:
            return Clutch::B1;
    }
}

const Clutch get_clutch_to_release(ProfileGearChange change) {
    switch(change) {
        case ProfileGearChange::ONE_TWO:
        case ProfileGearChange::FIVE_FOUR:
            return Clutch::B1;
        case ProfileGearChange::TWO_THREE:
        case ProfileGearChange::FOUR_THREE:
            return Clutch::K3;
        case ProfileGearChange::THREE_FOUR:
            return Clutch::B2;
        case ProfileGearChange::THREE_TWO:
            return Clutch::K2;
        case ProfileGearChange::FOUR_FIVE:
        case ProfileGearChange::TWO_ONE:
        default:
            return Clutch::K1;
    }
}

bool is_shifter_in_valid_drive_pos(ShifterPosition p) {
    bool ret = true;
    switch (p) {
        case ShifterPosition::P:
        case ShifterPosition::P_R:
        case ShifterPosition::N:
        case ShifterPosition::R_N:
            ret = false;
            break;
        default: // Already true (All other positions)
            break;
    }
    return ret;
}