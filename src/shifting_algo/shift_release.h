#ifndef __SHIFT_RELEASE_H__
#define __SHIFT_RELEASE_H__

#include "s_algo.h"

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
    uint16_t torque_adder = 0;
    int16_t pre_shift_trq = 0;
    float filling_adder = 0;
    float filling_trq_reducer = 0;

    float sports_trq_req_adder = 1.0;
    bool trq_req = true;
    
    int mod_time_phase_0 = -1;

    uint8_t subphase_mod = 0;
    uint8_t subphase_shift = 0;

    uint16_t ts_phase_mod = 0;
    uint16_t ts_phase_shift = 0;
    uint16_t trq_req_start_time = 0;
    uint16_t trq_req_end_time = 0;
    uint16_t last_trq_req = 0;

    float sports_factor=1.0;
    float trq_request_target = 0;

    uint16_t rpm_shift_phase_3 = 0;

    int torque_at_new_clutch = 0;
    bool torque_req_en = false;
};

#endif