#ifndef __SHIFT_CROSSOVER_H__
#define __SHIFT_CROSSOVER_H__

#include "s_algo.h"
#include "firstorder_average.h"

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
    uint16_t trq_req_start_time = 0;
    uint16_t trq_req_ramp_trq = 0;
    uint16_t trq_req_end_v = 0;
    uint16_t trq_req_end_time = 0;

    uint8_t prefill_phase_mod = 0;
    uint8_t prefill_phase_shift = 0;

    uint16_t ts_phase_mod = 0;
    uint16_t ts_phase_shift = 0;

    int momentum_adder = 0;
    bool torque_req_en = false;
    int trq_ramp_down_time = 0;
    int trq_ramp_up_time = 0;
    int trq_req_last_ramp = 0;
};

#endif