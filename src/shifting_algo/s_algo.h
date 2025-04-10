#ifndef __S_ALGO_H__
#define __S_ALGO_H__

#include "../src/common_structs.h"
#include "../pressure_manager.h"
#include "torque_converter.h"

struct TorqueRequstData {
    TorqueRequestControlType ty;
    TorqueRequestBounds bounds;
    float amount;
};

const uint8_t STEP_RES_CONTINUE = 0;
const uint8_t STEP_RES_FAILURE = 0xFE; // Shift failed. Abort!!
const uint8_t STEP_RES_END_SHIFT = 0xFF;


typedef struct {
    AbstractProfile* profile;
    int MOD_MAX;
    int SPC_MAX;
    
    uint16_t targ_time;
    GearChange change;
    Clutch applying;
    Clutch releasing;
    GearboxGear curr_g;
    GearboxGear targ_g;
    CircuitInfo inf;
    uint16_t release_spring_on_clutch;
    uint16_t release_spring_off_clutch;
    PrefillData prefill_info;
    ShiftCharacteristics chars;
    ShiftClutchData* ptr_r_clutch_speeds;
    ShiftClutchData* ptr_r_pre_clutch_speeds;
    ShiftPressures*  ptr_prev_pressures;
    ShiftPressures* ptr_w_pressures;
    TorqueRequstData* ptr_w_trq_req;
    PressureStageTiming maxp_info;
    TorqueConverter* tcc;
} ShiftInterfaceData;

class ShiftingAlgorithm {
public:
    ShiftingAlgorithm(ShiftInterfaceData* data) {
        this->sid = data;
    }
    virtual ~ShiftingAlgorithm() = default;
    ShiftAlgoFeedback get_diag_feedback(uint8_t phase_id);

    /**
     * SPECIAL RETURN VALUES
     * STEP_RES_CONTINUE - Continue in the same phase
     * STEP_RES_END_SHIFT - Finish shift
     * 1..254 - The next phase ID to run
     */
    virtual uint8_t step(
        uint8_t phase_id,
        uint16_t abs_input_torque,
        bool stationary,
        bool is_upshift,
        uint16_t phase_elapsed,
        uint16_t total_elapsed,
        PressureManager* pm,
        SensorData* sd
    ) = 0;

    void reset_all_subphase_data();
    virtual uint8_t max_shift_stage_id() = 0;

    protected:
        ShiftInterfaceData* sid;
        uint8_t subphase_mod = 0;
        uint16_t ts_subphase_mod = 0;
        uint8_t subphase_shift = 0;
        uint16_t ts_subphase_shift = 0;

        void inc_subphase_mod(uint16_t phase_elapsed_now);
        void inc_subphase_shift(uint16_t phase_elapsed_now);

        uint16_t elapsed_subphase_shift(uint16_t phase_elapsed_now);
        uint16_t elapsed_subphase_mod(uint16_t phase_elapsed_now);

        void set_trq_request_val(uint16_t v);
        void disable_trq_request(uint16_t total_elapsed);
        void trigger_trq_request(uint16_t total_elapsed);
        uint16_t get_trq_req_ramp_val(uint16_t total_elapsed, uint16_t ramp_down_time, uint16_t ramp_up_time);
        bool trq_request_is_end_ramp();
        uint16_t threshold_rpm = 0;
        float inertia = 0;

        // ------ TORQUE REQUEST STUFF ------
        bool request_trigger = false;
        uint16_t torque_request_targ = 0;
        uint16_t start_timestamp = 0;
        uint16_t end_timestamp = 0;


};

// Helper functions
namespace ShiftHelpers {
    float calcualte_abs_engine_inertia(uint8_t shift_idx, uint16_t engine_rpm, uint16_t input_rpm);
    float get_shift_intertia(uint8_t shift_idx);
    uint16_t ms_till_target_on_rpm(int target, int d_on_clutch, int rpm_on_clutch);
    uint16_t calc_output_mod_pressure(uint8_t shift_idx, uint16_t p_shift, uint16_t p_mod, uint16_t hydr_max);
    uint16_t get_threshold_rpm(uint8_t shift_idx, uint16_t abs_trq, uint8_t ramp_cycles);
}

#endif