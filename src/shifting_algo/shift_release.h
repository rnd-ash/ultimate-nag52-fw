#ifndef __SHIFT_RELEASE_H__
#define __SHIFT_RELEASE_H__

#include "s_algo.h"
#include "firstorder_average.h"

class ReleasingShift : public ShiftingAlgorithm {
public:
    explicit ReleasingShift(ShiftInterfaceData* data);
    ~ReleasingShift() override;
    uint8_t step_internal(
        bool stationary,
        bool is_upshift
    ) override;
    uint8_t max_shift_stage_id() override;

protected:
    uint16_t max_p_mod_pressure() override;
    bool is_release_shift() override {return true; }

private:
    uint16_t cycles_high_filling = 0;
    uint16_t cycles_ramp_filling = 0;
    uint16_t cycles_low_filling = 0;

    uint16_t cycles_mod_ramp_to_sync = 0;
    uint16_t cycles_mod_hold_sync = 0;



    float freeing_trq = 0;
    float loss_torque = 0;
    float loss_torque_tmp = 0;
    uint16_t torque_adder = 0;

    float calculate_freeing_trq_multiplier();
    void phase_fill_release_spc();
    uint8_t phase_fill_release_mpc();
    uint8_t phase_overlap();

    short first_order_filter_in_place(uint16_t percentage, short new_value, short last_filtered_val);

    uint16_t calc_sync_mod_pressure();
    short calc_shifting_momentum();
    short calc_sync_torque_new_clutch();
    uint16_t calc_mod_overlap();
    int16_t calc_release_clutch_p_signed(int trq, CoefficientTy coef);
    uint16_t calc_threshold_rpm_2();
    uint16_t calc_cycles_mod_phase1();
    uint16_t calc_cycles_mod_phase2();   
    float spc_ramp_val = 0;
    float p_overlap_begin = 0;
    float overlap_torque = 0;
    uint8_t fill_1_mpc_cycles = 0;

    int spc_step_adder = 0;
    int spc_wait_adder = 0;
    int minimum_mod_reduction_trq = 0;
};

#endif