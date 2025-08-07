#include "common_structs_ops.h"

uint8_t fwd_gearchange_egs_map_lookup_idx(GearChange req) {
    uint8_t ret = 0xFF;
    // TODO - Handle invalid request
    if (GearChange::_IDLE < req && req <= GearChange::_5_4) {
        ret = (uint8_t)req - 1;
    }
    return ret;
}

Clutch get_clutch_to_apply(GearChange change) {
    switch(change) {
        case GearChange::_1_2:
        case GearChange::_5_4:
            return Clutch::K1;
        case GearChange::_2_3:
            return Clutch::K2;
        case GearChange::_3_4:
        case GearChange::_3_2:
            return Clutch::K3;
        case GearChange::_4_3:
            return Clutch::B2;
        case GearChange::_4_5:
        case GearChange::_2_1:
        default:
            return Clutch::B1;
    }
}

Clutch get_clutch_to_release(GearChange change) {
    switch(change) {
        case GearChange::_1_2:
        case GearChange::_5_4:
            return Clutch::B1;
        case GearChange::_2_3:
        case GearChange::_4_3:
            return Clutch::K3;
        case GearChange::_3_4:
            return Clutch::B2;
        case GearChange::_3_2:
            return Clutch::K2;
        case GearChange::_4_5:
        case GearChange::_2_1:
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

uint8_t gear_to_idx_lookup(GearboxGear g) {
    uint8_t gear_idx = 0;
    switch(g) {
        case GearboxGear::First:
            gear_idx = 1;
            break;
        case GearboxGear::Second:
            gear_idx = 2;
            break;
        case GearboxGear::Third:
            gear_idx = 3;
            break;
        case GearboxGear::Fourth:
            gear_idx = 4;
            break;
        case GearboxGear::Fifth:
            gear_idx = 5;
            break;
        case GearboxGear::Reverse_First:
            gear_idx = 6;
            break;
        case GearboxGear::Reverse_Second:
            gear_idx = 7;
            break;
        case GearboxGear::Park:
        case GearboxGear::Neutral:
        case GearboxGear::SignalNotAvailable:
        default:
            gear_idx = 0;
            break;
    }
    return gear_idx;
}