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
    this->momentum_pid[2] = 0;
    this->trq_adder = 0;
}

ShiftAlgoFeedback ShiftingAlgorithm::get_diag_feedback(uint8_t phase_id) {
    return ShiftAlgoFeedback {
        .active = 1, // True
        .shift_phase = 1, // Always (Fix weirdness)
        .subphase_shift = this->subphase_shift,
        .subphase_mod = this->subphase_mod,
        .sync_rpm = this->threshold_rpm,
        .pid_torque = (int16_t)this->correction_trq,
        .adder_torque = (int16_t)this->trq_adder,
        .p_on = (uint16_t)this->p_apply_clutch,
        .p_off = (uint16_t)this->mod_sol_pressure,
        .s_off = (int16_t)this->sid->ptr_r_clutch_speeds->off_clutch_speed,
        .s_on = (int16_t)this->sid->ptr_r_clutch_speeds->on_clutch_speed,
        .s_turbine = (int16_t)this->sd->input_rpm,
        .s_targ = (int16_t)this->target_turbine_speed
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
    this->pm = pm;
    this->sd = sd;
    
    // Decrease our timers
    if (this->timer_mod > 0) {
        this->timer_mod -= 1;
    }
    if (this->timer_shift > 0) {
        this->timer_shift -= 1;
    }
    // Continuously check shift flags
    ShiftHelpers::calc_shift_flags(this->sid, this->sd);

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
    int targ_spc = this->set_p_apply_clutch_with_spring(this->calc_high_filling_p());
    if (0 == this->subphase_mod) {
        // Initial variables set
        this->subphase_mod += 1;
        // Release downshift only (EGS53)
        if (this->is_release_shift() && !upshifting) {
            this->timer_mod = interpolate_float(sd->atf_temp, 20, 3, -45, -10, InterpType::Linear);
        } else {
            this->timer_mod = 3;
        }
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
    this->shift_sol_pressure = this->correct_shift_shift_pressure(p_apply_clutch);

calc_mod:
    if (this->is_release_shift()) {
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
        this->timer_mod =  5 + interpolate_float(sd->atf_temp, 40, 5, 0, 40, InterpType::Linear);
        this->subphase_shift += 1;
    } else if (1 == this->subphase_shift) {
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, sid->SPC_MAX, this->timer_shift);
        if (this->timer_mod == 0) {
            ret = STEP_RES_NEXT;
            // End of max pressure (Shut of shift valve)
            pm->set_shift_circuit(sid->inf.shift_circuit, false);
        }
    }
    this->shift_sol_pressure = this->correct_shift_shift_pressure(this->p_apply_clutch);
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
    this->shift_sol_pressure = this->correct_shift_shift_pressure(this->p_apply_clutch);
    this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, pm->find_working_mpc_pressure(sid->targ_g), this->timer_shift);
    if (this->timer_shift == 0) {
        ret = STEP_RES_END_SHIFT;
    }
    return ret;
}

uint16_t ShiftingAlgorithm::calc_max_trq_on_clutch(uint16_t pressure, CoefficientTy coef) {
    uint16_t ret = 0;
    short p_corrected = pressure + this->centrifugal_force_on_clutch - sid->release_spring_on_clutch;
    if (p_corrected > 0) {
        ret = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_corrected, coef);
    }
    return ret;
}

const uint8_t freewheeling_factors[8] = {20, 100, 100, 100, 100, 100, 80, 120}; // RELEASE_CAL->freewheeling_factor
uint16_t ShiftingAlgorithm::calc_mod_with_filling_trq_and_freewheeling(int p_shift) {
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
uint16_t ShiftingAlgorithm::calc_mod_with_filling_trq(int p_shift) {
    uint16_t p = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, abs(trq_at_release_clutch), CoefficientTy::Release) + sid->release_spring_off_clutch;
    if (p > this->centrifugal_force_off_clutch) {
        p -= this->centrifugal_force_off_clutch;
    } else {
        p = 0;
    }
    return this->calc_mpc_sol_shift_ps(p_shift, p);
}

uint16_t ShiftingAlgorithm::calc_mpc_sol_shift_ps(int p_shift, int p_mod) {
    int p = ((int)p_shift * (int)HYDR_PTR->overlap_circuit_factor_spc[sid->inf.map_idx]) / 1000;
    p += (((int)p_mod * (int)HYDR_PTR->overlap_circuit_factor_mpc[sid->inf.map_idx]) / 1000);
    p +=  (int)HYDR_PTR->overlap_circuit_spring_pressure[sid->inf.map_idx];
    return MIN(MAX(0, p), sid->MOD_MAX);
}

uint16_t ShiftingAlgorithm::calc_mod_min_abs_trq(int p_shift) {
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

uint16_t ShiftingAlgorithm::set_p_apply_clutch_with_spring(int p) {
    short res = MAX(0, p + sid->release_spring_on_clutch - this->centrifugal_force_on_clutch);
    return MIN(res, sid->SPC_MAX);
}

uint16_t ShiftingAlgorithm::calc_low_filling_p() {
    uint16_t ret = 0;
    if (this->is_release_shift() && (GearChange::_3_2 == sid->change || GearChange::_2_1 == sid->change))  {
        ret = 0;
    } else {
        int adder = 0;
        // Add a bit more pressure depending on vehicle speed
        if (this->upshifting && !this->is_release_shift()) {
            int rpm_adder = interpolate_float(sd->engine_rpm, 0, 50, 0, 6000, InterpType::Linear);
            int trq_adder = interpolate_float(abs_input_trq, 0, 50, 0, 500, InterpType::Linear);
            adder = rpm_adder + trq_adder;
            if (race == sid->profile) {
                adder *= 2;
            }
        }
        ret = sid->prefill_info.low_fill_pressure_on_clutch + adder;
    }
    return ret;
}

uint16_t ShiftingAlgorithm::calc_high_filling_p() {
    uint16_t ret = 0;
    // Release downshift and 2-1/3-2
    if (this->is_release_shift() && (GearChange::_3_2 == sid->change ||  GearChange::_2_1 == sid->change))  {
        ret = 0;
    } else {
        uint16_t adder_1 = 0;
        if (sd->atf_temp < -10) {
            // Very cold filling
            adder_1 = 500;
        }
        ret = sid->prefill_info.fill_pressure_on_clutch + adder_1;
        ret = MIN(sid->SPC_MAX, ret);
    }
    return ret;
}

uint16_t ShiftingAlgorithm::correct_shift_shift_pressure(int16_t pressure) {
    // TODO - Move max_p to global constant so it can be referred in other functions
    uint16_t max_p = pm->get_max_shift_pressure(sid->inf.map_idx);
    // Bypass EEPROM adaptation offsets for shift circuits
    pressure += this->spc_p_offset;
    if (pressure < 0) {
        pressure = 0;
    }
    if (pressure > max_p) {
        pressure = max_p;
    }
    // P*1000 as shift_spc_gain is *1000
    return (uint16_t)(((pressure*1000) / HYDR_PTR->shift_spc_gain[sid->inf.map_idx]) + HYDR_PTR->shift_reg_spring_pressure);
}


short ShiftingAlgorithm::calc_correction_trq(ShiftStyle style, short momentum) {
    short intertia = ShiftHelpers::get_shift_intertia(sid->inf.map_idx);
    if (this->upshifting) {
        this->target_turbine_speed -= ((momentum*20)/intertia);
        this->target_turbine_speed = MAX(0, this->target_turbine_speed);
    } else {
        this->target_turbine_speed += ((momentum*20)/intertia);
    }
    short p = 0;
    short i = 0;
    short d = 0;
    //short t = 0;
    switch (style) {
        case ShiftStyle::Crossover_Up:
            p = CRS_CURRENT_SETTINGS.pid_p_val_upshift;
            i = CRS_CURRENT_SETTINGS.pid_p_val_upshift;
            d = CRS_CURRENT_SETTINGS.pid_p_val_upshift;
            break;
        case ShiftStyle::Crossover_Dn:
            p = CRS_CURRENT_SETTINGS.pid_p_val_downshift;
            i = CRS_CURRENT_SETTINGS.pid_p_val_downshift;
            d = CRS_CURRENT_SETTINGS.pid_p_val_downshift;
            break;
        case ShiftStyle::Release_Up:
            p = REL_CURRENT_SETTINGS.pid_p_val_upshift;
            i = REL_CURRENT_SETTINGS.pid_i_val_upshift;
            d = 0;
            break;
        case ShiftStyle::Release_Dn:
            p = REL_CURRENT_SETTINGS.pid_p_val_downshift;
            i = REL_CURRENT_SETTINGS.pid_i_val_downshift;
            d = 0;
            break;
    }
    int16_t error = ((int16_t)sd->input_rpm - this->target_turbine_speed);

    int32_t p_v = (p*error)/1000;
    momentum_pid[1] += error;
    momentum_pid[1] = MAX(INT16_MIN, MIN(INT16_MAX, momentum_pid[1]));
    
    // TODO - EGS has no anti-windup logic for its PID algorithm
    int32_t i_v = (i*momentum_pid[1])/1000;
    int32_t d_v = (d*(error-momentum_pid[0]))/1000;
    momentum_pid[0] = error;

    int32_t ret = MAX(INT16_MIN, MIN(INT16_MAX, p_v + i_v + d_v));
    return (short)ret;
}