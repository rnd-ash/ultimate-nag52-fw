#ifndef COMMON_STRUCTS_OPS_H
#define COMMON_STRUCTS_OPS_H

#include "common_structs.h"

ShiftStage next_shift_stage(ShiftStage now);
const Clutch get_clutch_to_apply(ProfileGearChange change);
const Clutch get_clutch_to_release(ProfileGearChange change);

bool is_shifter_in_valid_drive_pos(ShifterPosition p);

#endif