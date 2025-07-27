#ifndef __SHIFT_CROSSOVER_H__
#define __SHIFT_CROSSOVER_H__

#include "s_algo.h"
#include "firstorder_average.h"

class CrossoverShift : public ShiftingAlgorithm {
public:
    CrossoverShift(ShiftInterfaceData* data);
    ~CrossoverShift() override;
    uint8_t step_internal(
        bool stationary,
        bool is_upshift
    ) override;

    void calc_shift_flags(SensorData* sd, uint32_t* dest) override;
    uint8_t max_shift_stage_id() override;
    
    
private:
    uint8_t phase_fill();
    uint8_t phase_overlap();
    uint8_t phase_overlap2();
    
    uint16_t p_apply_overlap_begin = 0;

    uint16_t fun_0d86b4();
    uint16_t fun_0d8a10(uint16_t p_shift);
    uint16_t fun_0d8a66();

    uint16_t trq_req_val = 0;
    float trq_adder_2 = 0;

    uint8_t trq_req_timer = 0;
    bool trq_req_up_ramp = false;
    uint16_t torque_req_val = 0;
};

#endif