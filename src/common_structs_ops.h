#ifndef COMMON_STRUCTS_OPS_H
#define COMMON_STRUCTS_OPS_H

#include "common_structs.h"
#include "shifter/shifter.h"

Clutch get_clutch_to_apply(GearChange change);
Clutch get_clutch_to_release(GearChange change);

bool is_shifter_in_valid_drive_pos(ShifterPosition p);
uint8_t fwd_gearchange_egs_map_lookup_idx(GearChange req);
uint8_t gear_to_idx_lookup(GearboxGear g);
#endif