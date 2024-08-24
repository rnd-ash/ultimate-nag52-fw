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

    virtual uint8_t max_shift_stage_id() = 0;

    protected:
        ShiftInterfaceData* sid;
};

// Helper functions

namespace ShiftHelpers {
    float calcualte_abs_engine_inertia(uint8_t shift_idx, uint16_t engine_rpm, uint16_t input_rpm);
}

#endif