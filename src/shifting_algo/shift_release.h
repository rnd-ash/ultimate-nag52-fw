#ifndef __SHIFT_RELEASE_H__
#define __SHIFT_RELEASE_H__

#include "s_algo.h"
#include "firstorder_average.h"

class ReleasingShift : public ShiftingAlgorithm {
public:
    ReleasingShift(ShiftInterfaceData* data);
    ~ReleasingShift() override;
    uint8_t step_internal(
        bool stationary,
        bool is_upshift
    ) override;

    void calc_shift_flags(SensorData* sd, uint32_t* dest) override;
    uint8_t max_shift_stage_id() override;
    

private:
    uint16_t momentum_plus_maxtrq = 0;
    uint16_t momentum_plus_maxtrq_1 = 0;
    float freeing_trq = 0;
    float loss_torque = 0;
    uint16_t torque_adder = 0;

    uint16_t torque_req_val = 0;

    void phase_fill_release_spc();
    uint8_t phase_fill_release_mpc(SensorData* sd, bool is_upshift);

    uint16_t interp_2_ints(uint16_t percentage, uint16_t start, uint16_t end);

    uint8_t phase_overlap();

    uint16_t fun_0d85d8();
    bool trq_req_up_ramp = false;
    uint8_t trq_req_timer = 0;
    uint16_t low_f_p = 0;
    int16_t calc_release_clutch_p_signed(int trq, CoefficientTy coef);
};

#endif