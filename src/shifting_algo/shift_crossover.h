#ifndef SHIFT_CROSSOVER_H
#define SHIFT_CROSSOVER_H

#include "s_algo.h"

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
    uint16_t ramp_filling_trq_limit = 0;
    uint8_t phase_fill();
    uint8_t phase_overlap();
    uint8_t phase_overlap2();

    void fill_adapt();
    void overlap_adapt();
    void overlap2_adapt();
    int8_t calc_t_adapt_offset_adv(int8_t cycle_change);
    
    uint16_t p_apply_overlap_begin = 0;

    uint16_t calc_overlap_mod();
    uint16_t calc_overlap_mod_min(int p_shift);
    uint16_t calc_overlap2_mod();
    uint16_t max_p_mod_pressure() override;
    uint16_t fill_ramping_mod_p();
    uint16_t get_rpm_threshold(uint8_t shift_idx, uint8_t ramp_cycles);
    uint16_t get_rpm_threshold_trq_req_end(uint8_t shift_idx, uint8_t ramp_cycles);
    uint16_t get_trq_adder_map_val();
    uint16_t get_trq_boost_adder();
    int16_t calc_momentum_overlap_2();

    uint8_t trq_req_timer = 0;
    bool trq_req_up_ramp = false;
    bool fill_via_ramp = false;
    uint16_t torque_req_val = 0;
    int16_t torque_adapt_val = 0;
    
    uint16_t fill_time_adapt_timer = 0;

    uint16_t cycles_high_filling = 0;
    uint16_t cycles_ramp_to_low_filling = 0;
    uint16_t cycles_low_filling = 0;

    int8_t result_fill_time_adaptation = 0;

    uint16_t get_and_set_adapt_rpm_off_clutch();
    void offset_adapt_timer_by_clutch_delay();
};

#endif