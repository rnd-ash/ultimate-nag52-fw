#ifndef __SHIFT_RELEASE_H__
#define __SHIFT_RELEASE_H__

#include "s_algo.h"
#include "firstorder_average.h"

class ReleasingShift : public ShiftingAlgorithm {
public:
    ReleasingShift(ShiftInterfaceData* data);
    ~ReleasingShift() override;
    uint8_t step(
        uint8_t phase_id,
        uint16_t abs_input_torque,
        bool stationary,
        bool is_upshift,
        uint16_t phase_elapsed,
        uint16_t total_elapsed,
        PressureManager* pm,
        SensorData* sd
    ) override;

    uint8_t max_shift_stage_id() override;
    float filling_adder = 0;
    float filling_trq_reducer = 0;
    bool trq_ramp_up = false;
    float sports_trq_req_adder = 1.0;
    
    int mod_time_phase_0 = -1;
    int mod_time_phase_1 = -1;

    float sports_factor=1.0;

    int torque_at_new_clutch = 0;
    int torque_at_old_clutch = 0;
    int freeing_torque_calc = 0;
    int low_fill_p = 0;

    float trq_req_adder = 0;

    
};

#endif