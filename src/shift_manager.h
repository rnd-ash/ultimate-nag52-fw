#ifndef __SHIFT_MANAGER_H_
#define __SHIFT_MANAGER_H_

#include <common_structs.h>
#include "profiles.h"

class ShiftManager {

public:
    void perform_shift(ProfileGearChange shift_request,  SensorData* sensors, AbstractProfile* profile);
    void abort_shift();
private:
    ShiftData get_shift_data();
};

#endif