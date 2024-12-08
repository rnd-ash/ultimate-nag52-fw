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

void ShiftingAlgorithm::set_trq_request_val(uint16_t v) {
    this->torque_request_targ = v;
}

void ShiftingAlgorithm::disable_trq_request(uint16_t total_elapsed) {
    if (0 == this->end_timestamp) {
        this->end_timestamp = total_elapsed;
    }
}

void ShiftingAlgorithm::trigger_trq_request(uint16_t total_elapsed) {
    if (0 == this->start_timestamp) {
        this->start_timestamp = total_elapsed;
    }
}

uint16_t ShiftingAlgorithm::get_trq_req_ramp_val(uint16_t total_elapsed, uint16_t ramp_down_time, uint16_t ramp_up_time) {
    uint16_t ret = 0;
    if (0 != this->start_timestamp) {
        // Request has been triggered at some point
        if (0 != this->end_timestamp) {
            // We are ramping up (Finishing)
            int into_up_ramp = total_elapsed - this->end_timestamp;
            ret = interpolate_float(into_up_ramp, this->torque_request_targ, 0, 0, ramp_up_time, InterpType::Linear);
        } else {
            // We are ramping down or holding
            int into_down_ramp = total_elapsed - this->start_timestamp;
            ret = interpolate_float(into_down_ramp, 0, this->torque_request_targ, 0, ramp_down_time, InterpType::Linear);
        }
    }
    return ret;
}

bool ShiftingAlgorithm::trq_request_is_end_ramp() {
    return 0 != this->end_timestamp;
}