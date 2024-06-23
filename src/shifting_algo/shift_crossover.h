#ifndef __SHIFT_CROSSOVER_H__
#define __SHIFT_CROSSOVER_H__

#include "s_algo.h"

class CrossoverShift : public ShiftingAlgorithm {
public:
    CrossoverShift(ShiftInterfaceData* data);
    ~CrossoverShift() override;
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
private:
    bool do_high_filling = false;
    uint16_t torque_req_start_time = 0;
    int16_t torque_at_req_start = 0;
    int16_t trq_req_amount_at_end = 0;
};

#endif