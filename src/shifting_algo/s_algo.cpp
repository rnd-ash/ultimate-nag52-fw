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