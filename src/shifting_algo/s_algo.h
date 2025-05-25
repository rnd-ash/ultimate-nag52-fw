#ifndef __S_ALGO_H__
#define __S_ALGO_H__

#include "../src/common_structs.h"
#include "../pressure_manager.h"
#include "torque_converter.h"
#include "shift_flags.h"

struct TorqueRequstData {
    TorqueRequestControlType ty;
    TorqueRequestBounds bounds;
    float amount;
};

const uint8_t STEP_RES_CONTINUE = 0;
const uint8_t STEP_RES_NEXT = 0xFD;
const uint8_t STEP_RES_FAILURE = 0xFE; // Shift failed. Abort!!
const uint8_t STEP_RES_END_SHIFT = 0xFF;

enum class ShiftStyle {
    Crossover_Up,
    Crossover_Dn,
    Release_Up,
    Release_Dn,
};

typedef struct {
    AbstractProfile* profile;
    int MOD_MAX;
    int SPC_MAX;
    uint32_t shift_flags;
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
     * STEP_RES_NEXT - Goto next phase
     * 1..253 - The next phase ID to run
     */
    virtual uint8_t step_internal(
        bool stationary,
        bool is_upshift
    ) = 0;

    uint8_t step(
        uint8_t phase_id,
        uint16_t abs_input_torque,
        bool stationary,
        bool is_upshift,
        uint16_t phase_elapsed,
        uint16_t total_elapsed,
        PressureManager* pm,
        SensorData* sd
    );

    void reset_all_subphase_data();
    virtual uint8_t max_shift_stage_id() = 0;
    // Called when shift solenoid is opened
    virtual void calc_shift_flags(SensorData* sd, uint32_t* dest) = 0;

    protected:
        ShiftInterfaceData* sid;
        uint8_t subphase_mod = 0;
        uint16_t timer_mod = 0;
        uint8_t subphase_shift = 0;
        uint16_t timer_shift = 0;

        // EGS COMPATIBILITY
        int centrifugal_force_on_clutch;
        int centrifugal_force_off_clutch;

        // EGS compatibility vars (Makes it easier to translate original EGS assembly)
        int p_apply_clutch;
        int max_p_apply_clutch;
        int max_trq_apply_clutch;
        int max_trq_release_clutch;
        int filling_trq;
        int spc_step_adder = 0;
        int mpc_trq_reducer = 0;
        int trq_adder = 0;
        uint16_t abs_input_trq;
        
        int mod_sol_pressure;
        int shift_sol_pressure;
        uint8_t phase_id;

        PressureManager* pm;
        SensorData* sd;

        short momentum_start_turbine_rpm = 0;
        short momentum_start_output_rpm = 0;
        short correction_trq = 0;

        // Because EGS is weird, bleed and end of ctrl phases have same for either shift
        uint8_t phase_bleed(PressureManager* pm, bool is_upshift);
        uint8_t phase_maxp(SensorData* sd);
        uint8_t phase_end_ctrl();

        
        // EGS compatibility functions
        uint16_t calc_max_trq_on_clutch(uint16_t p_apply_clutch, CoefficientTy coef);
        uint16_t fun_0d820a(uint16_t p);
        uint16_t fun_0d83d4();
        uint16_t calc_mod_min_abs_trq(uint16_t p_shift);
        uint16_t calc_mod_with_filling_trq_and_freewheeling(uint16_t p_shift);
        uint16_t calc_mod_with_filling_trq(uint16_t p_shift);
        uint16_t calc_mpc_sol_shift_ps(uint16_t p_shift, uint16_t p_mod);
        void reset_for_next_phase();
        
        uint16_t calc_cycles_mod_phase1();
        uint16_t calc_cycles_mod_phase2(bool is_upshift);

        uint16_t set_p_apply_clutch_with_spring(uint16_t p);
        uint16_t clamp_p_apply_clutch(int p);

        short calc_correction_trq(ShiftStyle style, uint16_t momentum);
        short pid_iterate(int32_t p, int32_t i, int32_t d, int32_t new_value);
        short momentum_pid[2];
        short momentum_target = 0;
        uint16_t threshold_rpm = 0;
};

// Helper functions
namespace ShiftHelpers {
    float calcualte_abs_engine_inertia(uint8_t shift_idx, uint16_t engine_rpm, uint16_t input_rpm);
    float get_shift_intertia(uint8_t shift_idx);
    uint16_t ms_till_target_on_rpm(int target, int d_on_clutch, int rpm_on_clutch);
    uint16_t calc_output_mod_pressure(uint8_t shift_idx, uint16_t p_shift, uint16_t p_mod, uint16_t hydr_max);
    uint16_t get_threshold_rpm(uint32_t shift_flags, uint8_t shift_idx, uint16_t abs_trq, uint8_t ramp_cycles);
}

#endif