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

struct TorqueRequestModel {
    // TARGET time to ramp down to request level
    uint16_t ramp_down_ms;
    // RAGET time to ramp up to 0 (No EGS request)
    uint16_t ramp_up_ms;
    // Recorded timestamp of the start of the down ramp
    uint16_t ramp_down_start_ms;
    // Recorded timestamp of the start of the up ramp
    uint16_t ramp_up_start_ms;
    // Momentum reduction target
    uint16_t targ;
    // Latch for if the down ramp has started
    bool down_triggered;
    // Latch for if the up ramp has started
    bool up_triggered;
};

const uint8_t STEP_RES_CONTINUE = 0;
const uint8_t STEP_RES_FAILURE = 0xFE; // Shift failed. Abort!!
const uint8_t STEP_RES_END_SHIFT = 0xFF;


typedef struct {
    AbstractProfile* profile;
    int MOD_MAX;
    int SPC_MAX;
    uint16_t targ_time;
    ProfileGearChange change;
    Clutch applying;
    Clutch releasing;
    GearboxGear curr_g;
    GearboxGear targ_g;
    CircuitInfo inf;
    uint16_t spring_on_clutch;
    uint16_t spring_off_clutch;
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

        void trq_req_set_val(uint16_t max_req);
        void trq_req_start_ramp(uint16_t total_elapsed);
        void trq_req_end_ramp(uint16_t total_elapsed);
        /**
         * Returns the current value of the active torque request (Motor torque reduction amount from indicated torque)
         * If this value is 0, then the request is not active.
         */
        uint16_t trq_req_get_val(uint16_t total_elapsed);
        /**
         * @brief Returns if the torque request is on its end ramp (Going from target to 0). This is needed for ignition
         * angle control on petrol engines.
         */
        bool trq_req_is_end_ramp();

        uint16_t threshold_rpm = 0;
        float inertia = 0;
        TorqueRequestModel trq_mdl;
};

// Helper functions
namespace ShiftHelpers {
    float calcualte_abs_engine_inertia(uint8_t shift_idx, uint16_t engine_rpm, uint16_t input_rpm);
    TorqueRequestModel trq_req_init_model(uint16_t ramp_down_ms, uint16_t ramp_up_ms);
}

#endif