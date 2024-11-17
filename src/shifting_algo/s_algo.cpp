#include "s_algo.h"

void ShiftingAlgorithm::inc_subphase_mod(uint16_t phase_elapsed_now) {
    this->subphase_mod += 1;
    this->ts_subphase_mod = phase_elapsed_now;
}

void ShiftingAlgorithm::inc_subphase_shift(uint16_t phase_elapsed_now) {
    this->subphase_shift += 1;
    this->ts_subphase_shift = phase_elapsed_now;
}

uint16_t ShiftingAlgorithm::elapsed_subphase_shift(uint16_t phase_elapsed_now) {
    return MAX(0, phase_elapsed_now - this->ts_subphase_shift);
}

uint16_t ShiftingAlgorithm::elapsed_subphase_mod(uint16_t phase_elapsed_now) {
    return MAX(0, phase_elapsed_now - this->ts_subphase_mod);
}

void ShiftingAlgorithm::reset_all_subphase_data() {
    this->subphase_mod = 0;
    this->subphase_shift = 0;
    this->ts_subphase_mod = 0;
    this->ts_subphase_shift = 0;
}

ShiftAlgoFeedback ShiftingAlgorithm::get_diag_feedback(uint8_t phase_id) {
    return ShiftAlgoFeedback {
        .active = 1, // True
        .shift_phase = (uint8_t)(phase_id + 1),
        .subphase_shift = this->subphase_shift,
        .subphase_mod = this->subphase_mod,
        .sync_rpm = this->threshold_rpm,
        .inertia = (int16_t)this->inertia,
        .p_on = (uint16_t)this->sid->ptr_w_pressures->on_clutch,
        .p_off = (uint16_t)this->sid->ptr_w_pressures->off_clutch,
        .s_off = (int16_t)this->sid->ptr_r_clutch_speeds->off_clutch_speed,
        .s_on = (int16_t)this->sid->ptr_r_clutch_speeds->on_clutch_speed,
    };
}

void ShiftingAlgorithm::trq_req_set_val(uint16_t max_req) {
    if (!this->trq_mdl.up_triggered) { // Only do this if we are not 'up' ramping
        this->trq_mdl.targ = max_req;
    }
}

void ShiftingAlgorithm::trq_req_start_ramp(uint16_t total_elapsed) {
    if (!this->trq_mdl.down_triggered) { // One time latch
        this->trq_mdl.ramp_down_start_ms = total_elapsed;
        this->trq_mdl.down_triggered = true;
    }
}

void ShiftingAlgorithm::trq_req_end_ramp(uint16_t total_elapsed) {
    if (!this->trq_mdl.up_triggered) { // One time latch
        this->trq_mdl.targ = this->trq_req_get_val(total_elapsed); // In case we didn't reach our initial target, get the value of the ramp and hold as our target
        this->trq_mdl.ramp_down_start_ms = 0; // Stop ramping down
        this->trq_mdl.ramp_up_start_ms = total_elapsed;
        this->trq_mdl.up_triggered = true;
    }
}

uint16_t ShiftingAlgorithm::trq_req_get_val(uint16_t total_elapsed) {
    uint16_t ret = 0;
    // Check up ramp first, as this would be triggered second
    if (this->trq_mdl.ramp_up_start_ms != 0) {
        // Ramping back up to torque
        int into = total_elapsed - this->trq_mdl.ramp_up_start_ms;
        ret = interpolate_float(into, this->trq_mdl.targ, 0, 0, this->trq_mdl.ramp_up_ms, InterpType::Linear);
        if (into >= this->trq_mdl.ramp_up_ms) {
            this->trq_mdl.ramp_up_start_ms = 0; // Disable the entire sytem once we come out of this ramp
            this->trq_mdl.ramp_down_start_ms = 0;
        }
    } 
    // No up ramp, then check if we should be ramping down or holding torque
    else if (this->trq_mdl.ramp_down_start_ms != 0) {
        // We are ramping down
        int into = total_elapsed - this->trq_mdl.ramp_down_start_ms;
        ret = interpolate_float(into, 0, this->trq_mdl.targ, 0, this->trq_mdl.ramp_down_ms, InterpType::Linear);
    } 
    return ret;
}

bool ShiftingAlgorithm::trq_req_is_end_ramp() {
    return (this->trq_mdl.ramp_up_start_ms != 0);
}