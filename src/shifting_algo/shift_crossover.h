#ifndef SHIFT_CROSSOVER_H
#define SHIFT_CROSSOVER_H

#include "s_algo.h"
#include "firstorder_average.h"

class CrossoverShift : public ShiftingAlgorithm {
public:
    explicit CrossoverShift(ShiftInterfaceData* data);
    ~CrossoverShift() override;
    uint8_t step_internal(
        bool stationary,
        bool is_upshift
    ) override;

    uint8_t max_shift_stage_id() override;
    
protected:
    bool is_release_shift() override {return false; }
private:
    uint16_t adaptation_trq_limit = 0;
    uint8_t phase_fill();
    uint8_t phase_overlap();
    uint8_t phase_overlap2();
    int16_t map_trq_adder = 0;
    uint16_t p_apply_overlap_begin = 0;

    uint16_t fun_0d86b4();
    uint16_t fun_0d8a10(uint16_t p_shift);
    uint16_t fun_0d8a66();
    uint16_t max_p_mod_pressure() override;
    uint16_t fill_ramping_mod_p();
    uint16_t get_rpm_threshold(uint8_t shift_idx, uint8_t ramp_cycles);
    uint16_t get_trq_adder_map_val();
    float trq_adder_2 = 0;
    float trq_adder_3 = 0;

    uint8_t trq_req_timer = 0;
    bool trq_req_up_ramp = false;
    uint16_t torque_req_val = 0;
};

#endif