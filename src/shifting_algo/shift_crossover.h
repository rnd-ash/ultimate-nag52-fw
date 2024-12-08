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

    int momentum_adder = 0;
    bool filling_mode_check = false;
    float adder_torque = 0;
    int decent_adder_torque = 0;
    int min_spc_clutch_allowed = 0;
    int torque_request_calc = 0;
    float sports_factor = 1.0;
    int max_torque_prefill = 0;
};

#endif