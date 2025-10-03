#include "shift_release.h"
#include <egs_calibration/calibration_structs.h>
#include "nvs/module_settings.h"

const uint8_t PHASE_BLEED            = 0;
const uint8_t PHASE_FILL_AND_RELEASE = 1;
const uint8_t PHASE_OVERLAP          = 2;
const uint8_t PHASE_MAX_PRESSURE     = 3;
const uint8_t PHASE_END_CONTROL      = 4;

#define SHIFT_SETTINGS REL_CURRENT_SETTINGS

ReleasingShift::ReleasingShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {
    this->trq_req_timer = 3; // 100ms for torque request down ramp
    this->cycles_high_filling = data->prefill_info.fill_time/20;
    this->cycles_ramp_filling = 3;
    this->cycles_low_filling = 5;
}

ReleasingShift::~ReleasingShift() {
}

uint8_t ReleasingShift::max_shift_stage_id() {
    return PHASE_END_CONTROL;
}

void ReleasingShift::calc_shift_flags(uint32_t* dest) {
    *dest = 0;
    if (sd->pedal_pos < 10) {
        if ((sid->targ_g < sid->curr_g) && (sid->targ_g == GearboxGear::Third || sid->targ_g == GearboxGear::Fourth)) {
            *dest |= SHIFT_FLAG_COAST_54_43;
        }
        *dest |= SHIFT_FLAG_COAST;
    }
    if (sid->change == GearChange::_1_2 || sid->change == GearChange::_3_2 || sid->change == GearChange::_4_3) {
        *dest |= SHIFT_FLAG_FREEWHEELING;
    }
}

uint16_t ReleasingShift::high_fill_pressure() {
    uint16_t base = sid->prefill_info.fill_pressure_on_clutch;
    return base;
}

uint16_t ReleasingShift::calc_threshold_rpm_2(uint8_t cycles) {
    int ret = 0;
    if ((sid->shift_flags & SHIFT_FLAG_COAST) == 0) {
        float torque_avg = (this->freeing_trq + this->max_trq_apply_clutch)/2;
        float torque_min = MIN(this->freeing_trq, torque_avg);
        float torque = torque_min + this->max_trq_apply_clutch;
        // Number of EGS cycles (20ms):
        // 1 20ms. Calc Trq req
        // 2 20ms. Tx Trq req
        // 3 20ms. Engine to implement Trq req
        float cycles_can = 3.0;
        float inertia = ShiftHelpers::get_shift_intertia(sid->inf.map_idx);
        float threshold = torque * (float)(cycles + (cycles_can*2)) * (float)MECH_PTR->turbine_drag[sid->inf.map_idx] / inertia;

        ret = MAX(threshold, SHIFT_SETTINGS.clutch_stationary_rpm);
    } else if ((sid->shift_flags & SHIFT_FLAG_FREEWHEELING) == 0) {
        ret = SHIFT_SETTINGS.clutch_stationary_rpm;
    } else {
        ret = 25;
    }
    return MAX(0, ret);
}

uint8_t ReleasingShift::step_internal(
    bool stationary,
    bool is_upshift
) {
    uint8_t ret = STEP_RES_CONTINUE;
    // Set ramp value on first iteration
    if (this->spc_ramp_val == 0) {
        this->spc_ramp_val = 8;
        // Also set SPC offset
        // TODO - We should set this in profile configuration
        int adder_rpm = interpolate_float(sd->input_rpm, 0, 500, 2000, 5000, InterpType::Linear);
        int adder_profile = 0;

        if (manual == sid->profile) {
            adder_profile = interpolate_float(sd->pedal_pos, 0, 1000, 10, 250, InterpType::Linear);
            adder_rpm *= 2;
        } else if (race == sid->profile)  {
            adder_profile = interpolate_float(sd->pedal_pos, 0, 2500, 10, 250, InterpType::Linear);
            adder_rpm *= 1.5;
        } else {
            // Auto profiles
            adder_profile = interpolate_float(sid->chars.target_shift_time, 0, 500, 800, 100, InterpType::Linear);
        }
        this->spc_p_offset = adder_rpm + adder_profile;
    }


    if (phase_id == PHASE_BLEED) {
        ret = this->phase_bleed(pm, is_upshift);
        calc_shift_flags(&sid->shift_flags);
    } else if (phase_id == PHASE_FILL_AND_RELEASE) {
        this->phase_fill_release_spc(is_upshift);
        ret = this->phase_fill_release_mpc(is_upshift);
    } else if (phase_id == PHASE_OVERLAP) {
        ret = this->phase_overlap(is_upshift);
    } else if (phase_id == PHASE_MAX_PRESSURE) {
        ret = this->phase_maxp(sd);
    } else if (phase_id == PHASE_END_CONTROL) {
        ret = this->phase_end_ctrl();
    } else {
        ret = STEP_RES_END_SHIFT; // WTF? Should never happen
    }

    // Do torque request stuff here
    this->torque_req_out = 0;
    if (sd->indicated_torque > sd->min_torque && sd->converted_torque > sd->min_torque && sd->engine_rpm > 1100) {
        // LIMIT TORQUE - Max torque clutch exceeded
        bool emergency_limit = false;
        int intervension_out = 0;
        int idx = sid->inf.map_idx;
        if (idx >= 4) {
            idx -= 4;
        }
        int max_trq_off = MECH_PTR->max_torque_off_clutch[idx];
        int max_trq_on = MECH_PTR->max_torque_on_clutch[idx];
        if (this->phase_id == PHASE_FILL_AND_RELEASE && this->subphase_mod < 5 && abs_input_trq > max_trq_off) {
            emergency_limit = true;
            intervension_out = (abs_input_trq - max_trq_off) / sd->tcc_trq_multiplier;
        } else if ((this->phase_id == PHASE_FILL_AND_RELEASE && this->subphase_mod >= 5) || (this->phase_id > PHASE_FILL_AND_RELEASE && this->phase_id < PHASE_END_CONTROL)) {
            // Rest of fill and release, or overlap / max P phase
            if (abs_input_trq > max_trq_on) {
                intervension_out = MAX(
                    this->freeing_trq / sd->tcc_trq_multiplier,
                    (abs_input_trq - max_trq_on) / sd->tcc_trq_multiplier
                );
            } else {
                intervension_out = this->freeing_trq / sd->tcc_trq_multiplier;
            }
        }

        if (emergency_limit) {
            this->torque_req_out = intervension_out;
        } else {
            if (trq_req_up_ramp) {
                // Up ramp
                this->torque_req_val = linear_ramp_with_timer(this->torque_req_val, 0, this->trq_req_timer);
                if (this->trq_req_timer > 0) {
                    this->trq_req_timer -= 1;
                }
                
            } else if (trq_req_down_ramp) {
                // Down ramp or holding
                this->torque_req_val = linear_ramp_with_timer(this->torque_req_val, intervension_out, this->trq_req_timer);
                if (this->trq_req_timer > 0) {
                    this->trq_req_timer -= 1;
                }
            }
            // Disable torque requests past a certain speed
            if (sd->output_rpm > SHIFT_SETTINGS.output_rpm_disable_trq_req && sid->change != GearChange::_4_3) {
                this->torque_req_val = 0;
            }
        }
        this->torque_req_out = this->torque_req_val;
    }

    // Output to CAN
    if (0 != torque_req_out) {
        torque_req_out = MIN(torque_req_out, sd->indicated_torque);
        sid->ptr_w_trq_req->amount = sd->indicated_torque - torque_req_out;
        sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
        sid->ptr_w_trq_req->ty =  this->trq_req_up_ramp ? TorqueRequestControlType::BackToDemandTorque : TorqueRequestControlType::NormalSpeed;
    } else {
        // No request
        sid->ptr_w_trq_req->ty = TorqueRequestControlType::None;
        sid->ptr_w_trq_req->amount = 0;
        sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
    }

    return ret;
}

void ReleasingShift::phase_fill_release_spc(bool is_upshift) {
    this->threshold_rpm = calc_threshold_rpm_2(4);
    if (0 == this->subphase_shift) {
        // Var set
        this->timer_shift = this->cycles_high_filling;
        this->subphase_shift += 1;
    } else if (1 == this->subphase_shift) {
        // high 
        this->max_trq_apply_clutch = 0;
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(this->high_fill_pressure());
        if (0 == this->timer_shift) {
            this->timer_shift = this->cycles_ramp_filling;
            this->subphase_shift += 1;
            this->low_f_p = sid->prefill_info.low_fill_pressure_on_clutch;
        }
    } else if (2 == this->subphase_shift) {
        // Ramp to low filling
        this->max_trq_apply_clutch = 0;
        int targ = this->set_p_apply_clutch_with_spring(this->low_f_p);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        if (0 == this->timer_shift) {
            this->timer_shift = this->cycles_low_filling;
            this->subphase_shift += 1;
        }
    } else if (3 == this->subphase_shift) {
        // Low filling pressure.
        this->max_trq_apply_clutch = 0;
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(this->low_f_p);
        if (0 == this->timer_shift) {
            this->subphase_shift += 1; // Next subphase has no time!
        }
    } else if (4 == this->subphase_shift) {
        // Wait
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(this->low_f_p);
        this->max_trq_apply_clutch = this->calc_max_trq_on_clutch(this->p_apply_clutch, CoefficientTy::Sliding);
        if (
            sid->shift_flags & SHIFT_FLAG_COAST || // Coasting
            // Has not moved yet to completion
            (sid->ptr_r_clutch_speeds->on_clutch_speed < SHIFT_SETTINGS.clutch_stationary_rpm) ||
            // Off clutch has not released and at the end of our filling time
            // MAX(0, sid->ptr_r_clutch_speeds->off_clutch_speed) < SHIFT_SETTINGS.clutch_stationary_rpm && 
            (this->subphase_mod > 3)
        ) {
            this->subphase_shift += 1;
        }
    } else if (5 == this->subphase_shift) {
        // Ramping until RPM threshold
        this->spc_step_adder += this->spc_ramp_val;
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(this->low_f_p + this->spc_step_adder);
        this->max_trq_apply_clutch = this->calc_max_trq_on_clutch(this->p_apply_clutch, CoefficientTy::Sliding);
        if (
            (MAX(0,sid->ptr_r_clutch_speeds->off_clutch_speed) > SHIFT_SETTINGS.clutch_stationary_rpm && ((sid->shift_flags & SHIFT_FLAG_FREEWHEELING) == 0)) ||
            (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm)
        ) {
            this->subphase_shift += 1;
        }
    } else if (6 == this->subphase_shift) {
        this->spc_step_adder += this->spc_ramp_val / 2.0;
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(this->low_f_p + this->spc_step_adder);
        this->max_trq_apply_clutch = this->calc_max_trq_on_clutch(this->p_apply_clutch, CoefficientTy::Sliding);
    }
    // Faster flare recovery
    if (this->subphase_shift >= 4 && sid->ptr_r_clutch_speeds->on_clutch_speed < -SHIFT_SETTINGS.clutch_stationary_rpm) {
        this->spc_p_offset += 20;
    }
    if (this->subphase_mod >= 4 && MAX(0,sid->ptr_r_clutch_speeds->off_clutch_speed) < SHIFT_SETTINGS.clutch_stationary_rpm && ((sid->shift_flags & SHIFT_FLAG_COAST) == 0)) {
        this->spc_p_offset += 2;
    }
    //if (sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
    //    sid->tcc->shift_start();
    //}
    // Write pressure
    this->shift_sol_pressure = pressure_manager->correct_shift_shift_pressure(sid->inf.map_idx, this->p_apply_clutch + spc_p_offset);
}

uint8_t ReleasingShift::phase_fill_release_mpc(bool is_upshift) {
    uint8_t ret = STEP_RES_CONTINUE;
    // Freeing torque, multiplied by scalar based on pedal position
    this->freeing_trq = MIN(abs_input_trq, (float)pm->find_freeing_torque(sid->change, abs_input_trq, sd->output_rpm)*this->calculate_freeing_trq_multiplier(is_upshift));
    if (0 == this->subphase_mod) {
        // Var setting
        this->timer_mod = this->calc_cycles_mod_phase1();
        this->subphase_mod += 1;
    } else if (1 == this->subphase_mod) {
        this->filling_trq = MAX(30, abs_input_trq);
        int p = MAX(0, this->calc_release_clutch_p_signed(this->filling_trq, CoefficientTy::Release) + (int)sid->release_spring_off_clutch - this->centrifugal_force_off_clutch);
        this->mod_sol_pressure = this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p);
        if (0 == this->timer_mod) {
            this->timer_mod = this->calc_cycles_mod_phase2(is_upshift);
            this->subphase_mod += 1;
        }
    } else if (2 == this->subphase_mod) {
        int trq = MAX(0, (abs_input_trq - this->freeing_trq + 0));
        int p = MAX(0, this->calc_release_clutch_p_signed(trq, CoefficientTy::Sliding) + (int)sid->release_spring_off_clutch - this->centrifugal_force_off_clutch);
        int targ = this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p);

        this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, targ, this->timer_mod);
        if (
            (0 == this->timer_mod) ||
            (sid->ptr_r_clutch_speeds->off_clutch_speed > SHIFT_SETTINGS.clutch_stationary_rpm &&
            sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm) ||
            (sid->ptr_r_clutch_speeds->on_clutch_speed < SHIFT_SETTINGS.clutch_stationary_rpm)
        ) {
            // Next phase
            this->subphase_mod += 1;
            this->loss_torque = 0;
            this->loss_torque_tmp = 0;
        }
    } else if (3 == this->subphase_mod) {
        // Reducing until off clutch releases

        // Multiplier for loss factor
        float loss_multi = interpolate_float(sd->pedal_pos, 1.0, 3.0, 50, 200, InterpType::Linear);
        float loss_factor = (0.5*loss_multi) * this->loss_torque;
        // In Nm/Step
        float adder_step = interpolate_float(sid->chars.target_shift_time, 1.0, 3.0, 500, 100, InterpType::Linear)*loss_multi;
        this->loss_torque += adder_step + loss_factor;

        int trq = (int)this->abs_input_trq - (int)this->freeing_trq + this->trq_adder - (int)this->loss_torque;
        int p = MAX(0, this->calc_release_clutch_p_signed(trq, CoefficientTy::Sliding) + (int)sid->release_spring_off_clutch - this->centrifugal_force_off_clutch);
        this->mod_sol_pressure = this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p);
        if (
            MAX(0,sid->ptr_r_clutch_speeds->off_clutch_speed) > SHIFT_SETTINGS.clutch_stationary_rpm || 
            trq < -(SHIFT_SETTINGS.maximum_mod_reduction_trq) ||
            sid->ptr_r_clutch_speeds->on_clutch_speed < SHIFT_SETTINGS.clutch_stationary_rpm
        ) {
            this->subphase_mod += 1;
            this->momentum_start_turbine_rpm = sd->input_rpm;
            this->momentum_start_output_rpm = sd->output_rpm;
            this->momentum_plus_maxtrq = this->freeing_trq + this->max_trq_apply_clutch;
            this->momentum_plus_maxtrq_1 = this->momentum_plus_maxtrq;
            this->correction_trq = 0;
        }
    } else if (4 == this->subphase_mod) {
        // PID Correction to ramp the disengaging clutch at a sensible rate
        this->momentum_plus_maxtrq = this->freeing_trq + this->max_trq_apply_clutch;
        this->momentum_plus_maxtrq_1 = interp_2_ints(80, this->momentum_plus_maxtrq, this->momentum_plus_maxtrq_1);
        this->correction_trq = this->calc_correction_trq(is_upshift ? ShiftStyle::Release_Up : ShiftStyle::Release_Dn, this->momentum_plus_maxtrq_1);
        
        int trq = (int)this->abs_input_trq - (int)this->freeing_trq + this->trq_adder - (int)this->loss_torque + this->correction_trq;
        int p = MAX(0, this->calc_release_clutch_p_signed(trq, CoefficientTy::Sliding) + (int)sid->release_spring_off_clutch - this->centrifugal_force_off_clutch);
        this->mod_sol_pressure = this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p);
        if (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm) {
            this->timer_mod = 4; // 4+4 as seen in CAL
            this->subphase_mod += 1;
            // Start torque request to prevent clutch burn up on merge
            this->trq_req_down_ramp = true;
            this->trq_req_timer = 4; // 100ms for up ramp
        }

    } else if (5 == this->subphase_mod) {
        // Sync. phase
        short res = this->fun_0d4ed0();
        this->momentum_plus_maxtrq = linear_ramp_with_timer(this->momentum_plus_maxtrq, res, timer_mod);
        this->momentum_plus_maxtrq_1 = interp_2_ints(80, this->momentum_plus_maxtrq, this->momentum_plus_maxtrq_1);
        this->correction_trq = MIN(this->correction_trq, this->calc_correction_trq(is_upshift ? ShiftStyle::Release_Up : ShiftStyle::Release_Dn, this->momentum_plus_maxtrq_1));
        uint16_t targ = this->fun_0d85d8();
        this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, targ, this->timer_mod);
        if (0 == this->timer_mod || sid->ptr_r_clutch_speeds->on_clutch_speed < SHIFT_SETTINGS.clutch_stationary_rpm) {
            this->timer_mod = 4;
            this->subphase_mod += 1;
        }
    } else if (6 == this->subphase_mod) {
        this->momentum_plus_maxtrq = this->fun_0d4ed0();
        this->momentum_plus_maxtrq_1 = interp_2_ints(80, this->momentum_plus_maxtrq, this->momentum_plus_maxtrq_1);
        this->correction_trq = MIN(this->correction_trq, this->calc_correction_trq(is_upshift ? ShiftStyle::Release_Up : ShiftStyle::Release_Dn, this->momentum_plus_maxtrq_1));
        uint16_t targ = this->fun_0d85d8();
        this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, targ, this->timer_mod);
        if (0 == this->timer_mod || sid->ptr_r_clutch_speeds->on_clutch_speed < SHIFT_SETTINGS.clutch_stationary_rpm) {
            ret = PHASE_OVERLAP;
        }
    }
    return ret;
}

const uint8_t OVERLAP_TIMES[4] = {10, 7, 6, 5};
uint8_t ReleasingShift::phase_overlap(bool is_upshift) {
    uint8_t ret = STEP_RES_CONTINUE;
    if (0 == this->subphase_shift) {
        // Variable set
        this->p_overlap_begin = this->p_apply_clutch + centrifugal_force_on_clutch;
        this->timer_mod = 5;
        int idx = sid->inf.map_idx;
        if (idx >= 4) {
            idx -= 4;
        }
        this->timer_shift = OVERLAP_TIMES[idx];
        this->max_trq_apply_clutch = this->calc_max_trq_on_clutch(this->low_f_p, CoefficientTy::Release);
        this->overlap_torque = (sd->tcc_trq_multiplier * this->torque_req_val) + this->max_trq_apply_clutch;
        if (this->overlap_torque > this->freeing_trq) {
            this->overlap_torque = this->freeing_trq;
        }
        this->trq_req_up_ramp = true;
        this->trq_req_timer =  3;
        this->subphase_shift += 1;
    }
    int end = this->set_p_apply_clutch_with_spring(pm->p_clutch_with_coef(sid->targ_g, sid->applying, abs_input_trq, CoefficientTy::Release));
    this->p_apply_clutch = MAX(linear_ramp_with_timer(this->p_apply_clutch, end, this->timer_shift), this->p_overlap_begin - centrifugal_force_on_clutch);
    this->shift_sol_pressure = pressure_manager->correct_shift_shift_pressure(sid->inf.map_idx, this->p_apply_clutch + spc_p_offset);
    if (sid->change == GearChange::_4_3) {
        if (0 == this->timer_mod) {
            this->overlap_torque = linear_ramp_with_timer(this->overlap_torque, abs_input_trq, this->timer_shift);
        } else {
            this->max_trq_apply_clutch = this->calc_max_trq_on_clutch(this->low_f_p, CoefficientTy::Release);
            this->overlap_torque = (sd->tcc_trq_multiplier * this->torque_req_val) + this->max_trq_apply_clutch;
            if (this->overlap_torque > this->freeing_trq) {
                this->overlap_torque = this->freeing_trq;
            }
        }
    } else {
        this->max_trq_apply_clutch = this->calc_max_trq_on_clutch(this->p_apply_clutch, CoefficientTy::Release);
        this->overlap_torque = (sd->tcc_trq_multiplier * this->torque_req_val) + this->max_trq_apply_clutch;
    }
    this->mod_sol_pressure = this->calc_mod_overlap();
    if (this->timer_shift == 0) {
        ret = PHASE_MAX_PRESSURE;
    }
    return ret;
}

uint16_t ReleasingShift::calc_mod_overlap() {
    if (sid->change == GearChange::_3_2 || sid->change == GearChange::_2_1) {
        if (((sid->shift_flags & SHIFT_FLAG_COAST) != 0) && sd->pedal_pos  < 10) {
            int trq_req = (this->torque_req_val * sd->tcc_trq_multiplier);
            int trq = MAX(0 + this->correction_trq - this->loss_torque - trq_req, -SHIFT_SETTINGS.maximum_mod_reduction_trq);
            float p_mod = pm->p_clutch_with_coef_signed(sid->curr_g, sid->releasing, trq, CoefficientTy::Sliding) + sid->release_spring_off_clutch - centrifugal_force_off_clutch;
            p_mod = MAX(p_mod, 0);
            p_mod *= 0.8;
            int p_shift = MAX(0, this->p_overlap_begin - centrifugal_force_on_clutch);
            return this->calc_mpc_sol_shift_ps(p_shift, p_mod);
        } else {
            int trq = MAX(0, abs_input_trq + 0 + this->correction_trq - this->loss_torque);
            trq = MAX(trq, -SHIFT_SETTINGS.maximum_mod_reduction_trq);
            float p_mod = pm->p_clutch_with_coef_signed(sid->curr_g, sid->releasing, trq, CoefficientTy::Sliding) + sid->release_spring_off_clutch - centrifugal_force_off_clutch;
            p_mod *= 0.8;
            int p_shift = MAX(0, this->p_overlap_begin - centrifugal_force_on_clutch);
            return this->calc_mpc_sol_shift_ps(p_shift, p_mod);
        }
    } else {
        int trq = MAX(0, abs_input_trq + 0 + this->correction_trq - loss_torque - overlap_torque);
        int p = pm->p_clutch_with_coef_signed(sid->curr_g, sid->releasing, trq, CoefficientTy::Sliding) + sid->release_spring_off_clutch - centrifugal_force_off_clutch;
        p = MAX(0, p);
        return this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p);
    }

}

uint16_t ReleasingShift::max_p_mod_pressure() {
    if (sid->change == GearChange::_2_1 || sid->change == GearChange::_3_2) {
        int t = (abs_input_trq + 0 + this->correction_trq) - this->loss_torque;
        int trq_val = sd->tcc_trq_multiplier * this->torque_req_val;
        if (t < trq_val) {
            trq_val = 0;
        } else {
            trq_val = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, (t-trq_val), CoefficientTy::Release);
        }
        float p = MAX(0, trq_val + sid->release_spring_off_clutch - centrifugal_force_off_clutch);
        p *= 0.8; // field_1f
        int spc = MAX(0, sid->release_spring_on_clutch - this->centrifugal_force_on_clutch);
        return MIN(sid->MOD_MAX, this->calc_mpc_sol_shift_ps(spc, p));
    } else {
        float p_spc = pm->p_clutch_with_coef(sid->targ_g, sid->applying, abs_input_trq, CoefficientTy::Release);
        p_spc = MIN(sid->SPC_MAX, MAX(0, p_spc + sid->release_spring_on_clutch - centrifugal_force_on_clutch));
        p_spc *= sid->inf.pressure_multi_spc;
        float adder = 0;
        if (centrifugal_force_off_clutch < sid->release_spring_off_clutch) {
            adder = (sid->release_spring_off_clutch - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc;
        }
        float pressure = p_spc + adder + sid->inf.mpc_pressure_spring_reduction;
        return MIN(pressure, sid->MOD_MAX);
    }
}

uint16_t ReleasingShift::interp_2_ints(uint16_t percentage, uint16_t start, uint16_t end) {
    int x1 = ((int)percentage * (int)start) / 100;
    int x2 = (100 - percentage) * end;
    return (x1 + (x2/100));
}

const uint8_t momentum_factors[8] = {100, 100, 100, 100, 80, 80, 100, 100}; // RELEASE_CAL->field20_0x16
uint16_t ReleasingShift::fun_0d85d8() {

    float p_mod = 0;
    float v = MAX(0, abs_input_trq + this->correction_trq + this->torque_adder);
    
    float trq_on_c = (pm->release_coefficient() * (float)this->max_trq_apply_clutch) / pm->sliding_coefficient();
    p_mod = trq_on_c;

    p_mod = p_mod + (this->torque_req_val * sd->tcc_trq_multiplier);
    float uVar2 = this->freeing_trq * ((float)momentum_factors[sid->inf.map_idx] / 100.0);

    float uVar1 = (p_mod + this->freeing_trq) - uVar2;
    float uVar3 = this->freeing_trq;
    if (uVar1 <= this->freeing_trq) {
      uVar3 = uVar1;
    }
    if (v < uVar3) {
        p_mod = 0;
    } else {
        p_mod = MAX(0, this->calc_release_clutch_p_signed(v - uVar3, CoefficientTy::Sliding) + sid->release_spring_off_clutch - this->centrifugal_force_off_clutch);
    }

    return this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p_mod);
}

short ReleasingShift::fun_0d4ed0() {
    short ret = 0;
    float trq_on_c = (pm->release_coefficient() * (float)this->max_trq_apply_clutch) / pm->sliding_coefficient();
    trq_on_c += (this->torque_req_out * sd->tcc_trq_multiplier);
    
    float momentum_val = this->freeing_trq * ((float)momentum_factors[sid->inf.map_idx] / 100.0);
    
    float x = MIN((trq_on_c + this->freeing_trq) - momentum_val, this->freeing_trq);

    float trq_req_val = (this->torque_req_out * sd->tcc_trq_multiplier);

    ret = MAX(0, x + this->max_trq_apply_clutch - trq_req_val);

    return ret;
}

int16_t ReleasingShift::calc_release_clutch_p_signed(int trq, CoefficientTy coef) {
    return pm->p_clutch_with_coef_signed(sid->curr_g, sid->releasing, trq, coef);
}

float ReleasingShift::calculate_freeing_trq_multiplier(bool is_upshift) {
    float output = 1.0;

    if (!is_upshift) {
        float adder_pedal = interpolate_float(sd->pedal_smoothed->get_average(), 0.0, 0.3, 125.0, 250.0, InterpType::Linear);
        float adder_style = interpolate_float(sid->chars.target_shift_time, 0.5, 1.5, 1000, 100, InterpType::Linear);
        output = MIN(2.5, 1.0 + adder_pedal + adder_style);
    }

    return output;
}

uint16_t ReleasingShift::calc_cycles_mod_phase1() {
    uint16_t ret = 0;
    // 2->1 and 3->2 are set to 0 always
    if (sid->change != GearChange::_2_1 && sid->change != GearChange::_3_2) {
        float max_cycles = this->cycles_high_filling + this->cycles_ramp_filling + this->cycles_low_filling;
        float rpm_off_abs = abs(sid->ptr_r_clutch_speeds->on_clutch_speed);
        float calc = (rpm_off_abs * ShiftHelpers::get_shift_intertia(sid->inf.map_idx) / (float)MECH_PTR->turbine_drag[sid->inf.map_idx]);
        if (this->freeing_trq > 1) {
            calc /= this->freeing_trq;
        }
        ret = MAX(0, max_cycles - calc);
        if (sd->atf_temp < 30 && ret <= this->cycles_high_filling) {
            ret = this->cycles_high_filling;
        }
    }
    this->fill_1_mpc_cycles = ret;
    return ret;
}

uint16_t ReleasingShift::calc_cycles_mod_phase2(bool is_upshift) {
    uint16_t ret = 4;
    if (is_upshift) {
        int max = this->cycles_high_filling + this->cycles_ramp_filling;
        ret = MAX(4, max - this->fill_1_mpc_cycles);
    }
    return ret;
}