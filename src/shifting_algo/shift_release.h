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
        int16_t static_torque_no_reduction,
        bool stationary,
        bool is_upshift,
        uint16_t phase_elapsed,
        uint16_t total_elapsed,
        PressureManager* pm,
        SensorData* sd
    ) override;

    uint8_t max_shift_stage_id() override;
    uint16_t torque_adder = 0;
private:
    FirstOrderAverage<uint16_t>* trq_req_buffer;
};

#endif