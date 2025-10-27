#include "s_algo.h"
#include <egs_calibration/calibration_structs.h>
#include "nvs/module_settings.h"

void ShiftingAlgorithm::reset_all_subphase_data() {
    this->subphase_mod = 0;
    this->subphase_shift = 0;
    this->timer_mod = 0;
    this->timer_shift = 0;
    this->momentum_pid[0] = 0;
    this->momentum_pid[1] = 0;
}

ShiftAlgoFeedback ShiftingAlgorithm::get_diag_feedback(uint8_t phase_id) {
    return ShiftAlgoFeedback {
        .active = 1, // True
        .shift_phase = (uint8_t)(phase_id + 1),
        .subphase_shift = this->subphase_shift,
        .subphase_mod = this->subphase_mod,
        .sync_rpm = this->threshold_rpm,
        .pid_torque = (int16_t)this->correction_trq,
        .adder_torque = (int16_t)this->trq_adder,
        .p_on = (uint16_t)this->sid->ptr_w_pressures->on_clutch,
        .p_off = (uint16_t)this->sid->ptr_w_pressures->mod_sol_req,
        .s_off = (int16_t)this->sid->ptr_r_clutch_speeds->off_clutch_speed,
        .s_on = (int16_t)this->sid->ptr_r_clutch_speeds->on_clutch_speed,
    };
}

uint8_t ShiftingAlgorithm::step(
    uint8_t phase_id,
    uint16_t abs_input_torque,
    bool stationary,
    bool is_upshift,
    uint16_t phase_elapsed,
    uint16_t total_elapsed,
    PressureManager* pm,
    SensorData* sd
) {
    this->upshifting = is_upshift;
    // update EGS compatibility vars
    this->phase_id = phase_id;
    // Vars that are updated every cycle
    this->centrifugal_force_on_clutch = pm->calculate_centrifugal_force_for_clutch(sid->applying, sd->input_rpm, abs(sid->ptr_r_clutch_speeds->rear_sun_speed));
    this->centrifugal_force_off_clutch = pm->calculate_centrifugal_force_for_clutch(sid->releasing, sd->input_rpm, abs(sid->ptr_r_clutch_speeds->rear_sun_speed));
    // EGS compatibility vars updated every cycle
    this->abs_input_trq = abs_input_torque;
    this->trq_adder = 0; //pm->find_decent_adder_torque(sid->change, abs_input_torque, sd->output_rpm);
    this->pm = pm;
    this->sd = sd;
    
    // Decrease our timers
    if (this->timer_mod > 0) {
        this->timer_mod -= 1;
    }
    if (this->timer_shift > 0) {
        this->timer_shift -= 1;
    }

    // Sequence the inner shift logic
    uint8_t step_res = this->step_internal(stationary, is_upshift);
    if (STEP_RES_CONTINUE != step_res) {
        // We have switched phases, reset the timers
        this->reset_all_subphase_data();
    }

    // Update output variables
    sid->ptr_w_pressures->shift_sol_req = this->shift_sol_pressure;
    sid->ptr_w_pressures->mod_sol_req = this->mod_sol_pressure;
    if (phase_id != 0 && phase_id != this->max_shift_stage_id()) {
        sid->ptr_w_pressures->on_clutch = this->p_apply_clutch;
    } else {
        sid->ptr_w_pressures->on_clutch = 0;
    }

    return step_res;
}

uint8_t ShiftingAlgorithm::phase_bleed(PressureManager* pm) {
    uint8_t ret = STEP_RES_CONTINUE;
    this->trq_at_release_clutch = MAX(30, abs_input_trq);
    int targ_spc = this->set_p_apply_clutch_with_spring(this->high_fill_pressure());
    if (0 == this->subphase_mod) {
        // Initial variables set
        this->subphase_mod += 1;
        this->timer_mod = 5; // 100ms
    }
    if (1 == this->subphase_mod) {
        // End of phase!
        if (0 == this->timer_mod) {
            // Open the shift circuit
            pm->set_shift_circuit(sid->inf.shift_circuit, true);
            this->reset_for_next_phase();
            ret = STEP_RES_NEXT;
        }
    } else {
        // Invalid state!
        ret = STEP_RES_FAILURE;
        goto calc_mod;
    }
    this->p_apply_clutch = linear_ramp_with_timer(sid->SPC_MAX, targ_spc, this->timer_mod);
    this->shift_sol_pressure = pressure_manager->correct_shift_shift_pressure(sid->inf.map_idx, p_apply_clutch + spc_p_offset);

calc_mod:
    // if RELEASING_UPSHIFT || CROSSOVER_DOWNSHIFT
    if ((!this->upshifting && !this->is_release_shift()) || (this->upshifting && this->is_release_shift())) {
        if (GearChange::_2_3 == sid->change) {
            targ_spc *= 1.993;
        }
        this->mod_sol_pressure = this->calc_mod_with_filling_trq(targ_spc);
    } else {
        uint16_t mod_with_freewheeling = this->calc_mod_with_filling_trq_and_freewheeling(targ_spc);
        uint16_t uVar3 = this->calc_mod_min_abs_trq(targ_spc);
        this->mod_sol_pressure = MAX(mod_with_freewheeling, uVar3);
    }
    return ret;
}

uint8_t ShiftingAlgorithm::phase_maxp(SensorData* sd) {
    uint8_t ret = STEP_RES_CONTINUE;
    uint16_t targ_mpc = this->max_p_mod_pressure();
    if (0 == this->subphase_shift) {
        // Var set
        this->timer_shift = 5; // 100ms for ramp
        this->timer_mod = sid->maxp_info.hold_time/20;
        this->subphase_shift += 1;
    } else if (1 == this->subphase_shift) {
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, sid->SPC_MAX, this->timer_shift);
        if (this->timer_mod == 0) {
            ret = STEP_RES_NEXT;
            // End of max pressure (Shut of shift valve)
            pm->set_shift_circuit(sid->inf.shift_circuit, false);
        }
    }
    this->shift_sol_pressure = pressure_manager->correct_shift_shift_pressure(sid->inf.map_idx, this->p_apply_clutch + spc_p_offset);
    this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, targ_mpc, this->timer_mod);
    return ret;
}

uint8_t ShiftingAlgorithm::phase_end_ctrl() {
    uint8_t ret = STEP_RES_CONTINUE;
    // TODO
    if (0 == this->subphase_shift) {
        this->timer_shift = interpolate_float(sd->atf_temp, 75, 5, -20, 30, InterpType::Linear);
        this->subphase_shift += 1;
    }
    this->p_apply_clutch = sid->SPC_MAX;
    this->shift_sol_pressure = pressure_manager->correct_shift_shift_pressure(sid->inf.map_idx, this->p_apply_clutch + spc_p_offset);
    this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, pm->find_working_mpc_pressure(sid->targ_g), this->timer_shift);
    if (this->timer_shift == 0) {
        //sid->tcc->shift_end();
        ret = STEP_RES_END_SHIFT;
    }
    return ret;
}

uint16_t ShiftingAlgorithm::calc_max_trq_on_clutch(uint16_t p_apply_clutch, CoefficientTy coef) {
    uint16_t ret = 0;
    if (p_apply_clutch + this->centrifugal_force_on_clutch > sid->release_spring_on_clutch) {
        uint16_t p = p_apply_clutch + this->centrifugal_force_on_clutch - sid->release_spring_on_clutch;
        ret = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p, coef);
    }
    return ret;
}

const uint8_t freewheeling_factors[8] = {20, 100, 100, 100, 100, 100, 80, 120}; // RELEASE_CAL->freewheeling_factor
uint16_t ShiftingAlgorithm::calc_mod_with_filling_trq_and_freewheeling(uint16_t p_shift) {
    int p = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, abs(trq_at_release_clutch), CoefficientTy::Release) + sid->release_spring_off_clutch;
    if (p > this->centrifugal_force_off_clutch) {
        p = (p - this->centrifugal_force_off_clutch) * (freewheeling_factors[sid->inf.map_idx]);
        p /= 100; // Since freewheeling_factors is 0-100 not 0-1.0
    } else {
        p = 0;
    }
    return this->calc_mpc_sol_shift_ps(p_shift, p);
}

// FUN_d82d6
uint16_t ShiftingAlgorithm::calc_mod_with_filling_trq(uint16_t p_shift) {
    uint16_t p = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, abs(trq_at_release_clutch), CoefficientTy::Release) + sid->release_spring_off_clutch;
    if (p > this->centrifugal_force_off_clutch) {
        p -= this->centrifugal_force_off_clutch;
    } else {
        p = 0;
    }
    return this->calc_mpc_sol_shift_ps(p_shift, p);
}

// FUN_d8028
uint16_t ShiftingAlgorithm::calc_mpc_sol_shift_ps(uint16_t p_shift, uint16_t p_mod) {
    float p = 0;
    int p_s = ((int)p_shift * sid->inf.pressure_multi_spc_int) / 1000;
    int p_m = ((int)p_mod * sid->inf.pressure_multi_mpc_int) / 1000;

    p = p_s + p_m + sid->inf.mpc_pressure_spring_reduction;
    if (p <= 0) {
        p = 0;
    }
    if (p >= sid->MOD_MAX) {
        p = sid->MOD_MAX;
    }
    return p;
}

// FUN_8d832a
uint16_t ShiftingAlgorithm::calc_mod_min_abs_trq(uint16_t p_shift) {
    int p_shift_abs = pm->p_clutch_with_coef(sid->targ_g, sid->applying, this->abs_input_trq, CoefficientTy::Sliding) + sid->release_spring_on_clutch;
    p_shift_abs = MAX(0, p_shift_abs - this->centrifugal_force_on_clutch);

    if (this->centrifugal_force_on_clutch < p_shift + sid->release_spring_on_clutch) {
        p_shift = p_shift + sid->release_spring_on_clutch - centrifugal_force_on_clutch;
    }
    p_shift = MIN(MAX(p_shift_abs, p_shift), sid->SPC_MAX);

    int p_mod = 0;
    if (this->centrifugal_force_off_clutch < sid->release_spring_off_clutch) {
        p_mod = (sid->release_spring_off_clutch - this->centrifugal_force_off_clutch) * freewheeling_factors[sid->inf.map_idx];
        p_mod /= 100;
    }
    return this->calc_mpc_sol_shift_ps(p_shift, p_mod);
}

void ShiftingAlgorithm::reset_for_next_phase() {
    this->subphase_mod = 0;
    this->subphase_shift = 0;
}

uint16_t ShiftingAlgorithm::fun_0d83d4() {
    int p_shift = 0;
    if (this->centrifugal_force_on_clutch < sid->release_spring_on_clutch) {
        p_shift = 1000*this->p_apply_clutch/1000;
        p_shift += (1000-1000) * (this->sid->release_spring_on_clutch - this->centrifugal_force_on_clutch) / 1000;
    }
    int p_mod = MAX(0, pm->p_clutch_with_coef(sid->targ_g, sid->applying, trq_at_release_clutch, CoefficientTy::Release) + sid->release_spring_off_clutch - this->centrifugal_force_off_clutch);
    return this->calc_mpc_sol_shift_ps(p_shift, p_mod);
}

uint16_t ShiftingAlgorithm::set_p_apply_clutch_with_spring(uint16_t p) {
    uint16_t ret = 0;
    if ((int)p + (int)sid->release_spring_on_clutch < this->centrifugal_force_on_clutch) {
        ret = 0;
    } else {
        ret = (int)p + (int)sid->release_spring_on_clutch - this->centrifugal_force_on_clutch;
        // SPC reduction_P
    }
    return MIN(ret, sid->SPC_MAX);
}


short ShiftingAlgorithm::calc_correction_trq(ShiftStyle style, short momentum) {
    short intertia = ShiftHelpers::get_shift_intertia(sid->inf.map_idx);
    if (this->upshifting) {
        this->target_turbine_speed -= (momentum*20)/intertia;
    } else {
        this->target_turbine_speed += (momentum*20)/intertia;
    }
    this->target_turbine_speed = MAX(0, this->target_turbine_speed);

    short p = 0;
    short i = 0;
    short d = 0; // Always??
    //short t = 0;
    switch (style) {
        case ShiftStyle::Crossover_Up:
            if (sid->ptr_r_clutch_speeds->off_clutch_speed > 130 || sid->inf.map_idx != 0 || sd->atf_temp < 40) {
                p = 120;
                i = 4;
            } else {
                p = 75;
                i = 2;
            }
            break;
        case ShiftStyle::Release_Up:
            p = REL_CURRENT_SETTINGS.pid_p_val_upshift;
            i = REL_CURRENT_SETTINGS.pid_i_val_upshift;
            break;
        case ShiftStyle::Crossover_Dn:
            p = 200;
            i = 5;
            break;
        case ShiftStyle::Release_Dn:
            p = REL_CURRENT_SETTINGS.pid_p_val_downshift;
            i = REL_CURRENT_SETTINGS.pid_i_val_downshift;
            break;
    }
    float error = (sd->input_rpm - this->target_turbine_speed);
    // I component (index 0)
    this->momentum_pid[0] += ((float)(i)/1000.0) * error;
    if (this->momentum_pid[0] >= INT16_MAX) {
        this->momentum_pid[0] = INT16_MAX;
    } else if (this->momentum_pid[0] <= INT16_MIN) {
        this->momentum_pid[0] = INT16_MIN;
    }
    // P component (index 1) (EGS does not store this??...)
    float result = ((float)(p)/1000.0) * error + this->momentum_pid[0];
    if (result >= INT16_MAX) {
        result = INT16_MAX;
    } else if (result <= INT16_MIN) {
        result = INT16_MIN;
    }
    return (short)result;
}