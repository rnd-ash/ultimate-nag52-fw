#include "common_structs_ops.h"

ShiftStage next_shift_stage(ShiftStage now) {
    switch(now) {
        case ShiftStage::Bleed:
            return ShiftStage::Fill;
        case ShiftStage::Fill:
            return ShiftStage::Torque;
        case ShiftStage::Torque:
            return ShiftStage::Overlap;
        case ShiftStage::Overlap:
        case ShiftStage::MaxPressure:
        default:
            return ShiftStage::MaxPressure;
    }
}