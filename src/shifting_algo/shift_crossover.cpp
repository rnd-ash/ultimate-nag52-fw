#include "shift_crossover.h"
#include <egs_calibration/calibration_structs.h>

const uint8_t PHASE_BLEED            = 0;
const uint8_t PHASE_FILL             = 1;
const uint8_t PHASE_OVERLAP          = 2;
const uint8_t PHASE_OVERLAP2         = 3;
const uint8_t PHASE_MAX_PRESSURE     = 4;
const uint8_t PHASE_END_CONTROL      = 5;

CrossoverShift::CrossoverShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {
}

CrossoverShift::~CrossoverShift() {
}

uint8_t CrossoverShift::max_shift_stage_id() {
    return PHASE_END_CONTROL;
}

uint8_t FAC_TABLE[8] = {90, 90, 85, 70, 100, 100, 100, 100};
// P1 - IDX
// P2 - Cycles
uint16_t CrossoverShift::get_rpm_threshold(uint8_t shift_idx, uint8_t ramp_cycles) {
    float torque = this->get_trq_adder_map_val() + this->get_trq_boost_adder() + this->torque_req_val;
    float bVar1 = 6;
    float inertia = ShiftHelpers::get_shift_intertia(sid->inf.map_idx);
    float threshold = (torque*5*(ramp_cycles+(bVar1*2))) * (float)MECH_PTR->turbine_drag[sid->inf.map_idx] / inertia;
    threshold /= 10.0;
    return MAX(threshold, CRS_CURRENT_SETTINGS.clutch_stationary_rpm);
}

uint8_t CrossoverShift::step_internal(
    bool stationary,
    bool is_upshift
) {
    uint8_t ret = STEP_RES_CONTINUE;
    if (phase_id == PHASE_BLEED) {
        ret = this->phase_bleed(pm);
    } else if (phase_id == PHASE_FILL) {
        ret = this->phase_fill();
    } else if (phase_id == PHASE_OVERLAP) {
        ret = this->phase_overlap();
    } else if (phase_id == PHASE_OVERLAP2) {
        ret = this->phase_overlap2();
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
        int intervension_out = 0;
        if (this->phase_id >= PHASE_OVERLAP) {
            float multi_engine_trq = interpolate_float(sd->pedal_pos, &CRS_CURRENT_SETTINGS.trq_req_multi_pedal_pos, InterpType::Linear);
            float multi_rpm = interpolate_float(sd->input_rpm, &CRS_CURRENT_SETTINGS.trq_req_multi_input_rpm, InterpType::Linear);
            float out = (float)abs_input_trq * (multi_engine_trq*multi_rpm);
            intervension_out = out;
        }
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


uint8_t CrossoverShift::phase_fill() {
    uint8_t ret = STEP_RES_CONTINUE;
    uint16_t high_filling_p = this->calc_high_filling_p();
    uint16_t low_filling_p = this->calc_low_filling_p();
    if (0 == this->subphase_shift) {
        // Set vars
        this->timer_shift = sid->prefill_info.fill_cycles;
        this->subphase_shift += 1;
    }
    if (1 == this->subphase_shift) {
        // High filling
        this->p_apply_clutch = set_p_apply_clutch_with_spring(high_filling_p);
        if (0 == this->timer_shift) {
            this->timer_shift = 3; // FILL_CAL->fill_ramp_time
            // Roughly 2x drag torque
            this->adaptation_trq_limit = VEHICLE_CONFIG.engine_drag_torque/10.0;
            if (abs_input_trq < this->adaptation_trq_limit && sid->inf.map_idx < 5) {
                // Adaptation filling
                this->subphase_shift = 4;
            } else {
                // Non adaptation filling
                this->subphase_shift = 2;
            }
        }
        uint16_t p_mod_1 = this->calc_mod_with_filling_trq_and_freewheeling(this->p_apply_clutch);
        uint16_t p_mod_2 = this->calc_mod_min_abs_trq(low_filling_p);
        this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
    } 
    // Static filling (Torque too high for adaptation)
    else if (2 == this->subphase_shift) {
        // Ramp to low filling P
        uint16_t targ = this->set_p_apply_clutch_with_spring(low_filling_p);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        if (0 == this->timer_shift) {
            this->timer_shift = 5;
            this->subphase_shift += 1;
        }
        uint16_t p_mod_1 = this->calc_mod_with_filling_trq_and_freewheeling(this->p_apply_clutch);
        uint16_t p_mod_2 = this->calc_mod_min_abs_trq(low_filling_p);
        this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
    } else if (3 == this->subphase_shift) {
        // Low filling P
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(low_filling_p);
        uint16_t p_mod_1 = this->calc_mod_with_filling_trq_and_freewheeling(this->p_apply_clutch);
        uint16_t p_mod_2 = this->calc_mod_min_abs_trq(low_filling_p);
        this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
        if (0 == this->timer_shift) {
            ret = PHASE_OVERLAP;
        }
    }
    // Adaptation fill ramping
    
    // TODO - Abort on torque violations
    else if (4 == this->subphase_shift) {
        // Drop to 0 mBar for adapting to start (Pre ramping)
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(0);
        this->mod_sol_pressure = this->fill_ramping_mod_p();
        this->timer_shift = 18; // field 2a
        this->subphase_shift += 1;
    } else if (5 == this->subphase_shift) {
        // Low filling test
        uint16_t targ = this->set_p_apply_clutch_with_spring(600);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        this->mod_sol_pressure = this->fill_ramping_mod_p();
        if (0 == this->timer_shift) {
            this->timer_shift = 16; // field 2a
            this->subphase_shift += 1;
        }
    } else if (6 == this->subphase_shift) {
        // Higher filling test
        uint16_t targ = this->set_p_apply_clutch_with_spring(1100);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        this->mod_sol_pressure = this->fill_ramping_mod_p();
        if (0 == this->timer_shift) {
            ret = PHASE_OVERLAP;
        }
    }

    // Write Shift sol pressure
    this->shift_sol_pressure = this->correct_shift_shift_pressure(this->p_apply_clutch);

    // Adaptation skip test
    if (this->subphase_shift >= 4) {
        // Torque limit exceeded
        if (abs_input_trq > this->adaptation_trq_limit*1.5) {
            ret = PHASE_OVERLAP;
        }
    }
    return ret;
}

uint8_t CrossoverShift::phase_overlap() {
    uint8_t ret = STEP_RES_CONTINUE;
    this->trq_at_apply_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_apply_clutch, CoefficientTy::Sliding);
    this->trq_adder = 0;

    if (0 == subphase_shift) {
        this->p_apply_overlap_begin = MAX(0, this->p_apply_clutch - sid->release_spring_on_clutch + centrifugal_force_on_clutch);
        uint8_t interp_min = CRS_CURRENT_SETTINGS.overlap_cycles_low_trq;
        uint8_t interp_max = CRS_CURRENT_SETTINGS.overlap_cycles_high_trq;
        if (sid->change == GearChange::_1_2) {
            interp_min += CRS_CURRENT_SETTINGS.overlap_cycles_low_trq_adder_1_2;
            interp_max += CRS_CURRENT_SETTINGS.overlap_cycles_high_trq_adder_1_2;
        }
        int min_trq = VEHICLE_CONFIG.engine_drag_torque/5.0; // 2x drag torque real
        int max_trq = VEHICLE_CONFIG.engine_drag_torque; // 10x drag torque real
        this->timer_shift = interpolate_float(abs_input_trq,interp_min,interp_max, min_trq, max_trq, InterpType::Linear);

        uint8_t rpm_adder = interpolate_float(sd->input_rpm, &CRS_CURRENT_SETTINGS.overlap_cycles_adder_rpm, InterpType::Linear);
        this->timer_shift += rpm_adder;
        this->subphase_shift += 1;
    }
    uint16_t c_trq_apply = pm->p_clutch_with_coef(
        sid->targ_g,
        sid->applying,
        abs_input_trq + 0 - 0, // Trq adapt adder - Trq req adapt adder
        CoefficientTy::Sliding
    );
    uint16_t targ = MAX(
        this->set_p_apply_clutch_with_spring(c_trq_apply), 
        this->set_p_apply_clutch_with_spring(this->p_apply_overlap_begin)
    );
    this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
    uint16_t p_mod_1 = this->calc_overlap_mod();
    uint16_t p_mod_2 = this->calc_overlap_mod_min(this->p_apply_overlap_begin);
    this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
    if (
        this->timer_shift <= 1 || // ?? - EGS logic here, doesn't cmp to 0
        sid->ptr_r_clutch_speeds->off_clutch_speed > CRS_CURRENT_SETTINGS.clutch_stationary_rpm
    ) {
        // Next phase on clutch movement or timeout
        this->trq_adder_1 = MIN(0, (pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_apply_clutch, CoefficientTy::Sliding))-abs_input_trq);
        sid->tcc->shift_start(this->upshifting, false);
        ret = PHASE_OVERLAP2;
    }
    this->shift_sol_pressure = this->correct_shift_shift_pressure(this->p_apply_clutch);
    return ret;
}

uint16_t CrossoverShift::get_trq_adder_map_val() {
    float map_val = pm->find_decent_adder_torque(sid->change, this->abs_input_trq, sd->output_rpm);
    float multi = 1.0;
    if (this->upshifting) {
        if (race == sid->profile) {
            multi = CRS_CURRENT_SETTINGS.adder_trq_multi_race_up;
        } else if (manual == sid->profile) {
            multi = CRS_CURRENT_SETTINGS.adder_trq_multi_manual_up;
        } else {
            multi = CRS_CURRENT_SETTINGS.adder_trq_multi_normal_up;
        }
    } else {
        if (race == sid->profile) {
            multi = CRS_CURRENT_SETTINGS.adder_trq_multi_race_dn;
        } else if (manual == sid->profile) {
            multi = CRS_CURRENT_SETTINGS.adder_trq_multi_manual_dn;
        } else {
            multi = CRS_CURRENT_SETTINGS.adder_trq_multi_normal_dn;
        }
    }
    return MAX(0, map_val*multi);
}

uint16_t CrossoverShift::get_trq_boost_adder() {
    uint16_t ret = 0;
    // Use raw value (Not amplified, so shift is still smooth at the end)
    uint16_t map_val = pm->find_decent_adder_torque(sid->change, this->abs_input_trq, sd->output_rpm);
    int min = VEHICLE_CONFIG.engine_drag_torque/20.0;
    float boost_trq_adder = pm->sliding_coefficient() * (float)abs_input_trq / pm->release_coefficient();
    boost_trq_adder = MAX(0, boost_trq_adder - abs_input_trq);
    if (boost_trq_adder < min) {
        boost_trq_adder = min;
    }
    if (boost_trq_adder > map_val) {
        boost_trq_adder = map_val;
    }
    ret = boost_trq_adder;
    return ret;
}

uint8_t CrossoverShift::phase_overlap2() {
    uint8_t ret = STEP_RES_CONTINUE;
    this->trq_at_apply_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_apply_clutch, CoefficientTy::Sliding);

    // Overlap check can be skipped since it always compares timer to 0
    // Overlap 2 phase always starts when off clutch disengages
    // So, we can have just 1 check here for when to start the torque request
    if (!this->trq_req_down_ramp && abs(sid->ptr_r_clutch_speeds->off_clutch_speed) > CRS_CURRENT_SETTINGS.clutch_stationary_rpm) {
        // Start slowing down the engine (Clutch disengaged)
        this->trq_req_timer = 3;
        this->trq_req_down_ramp = true;
    }

    if (0 == subphase_shift) {
        uint8_t interp_min = CRS_CURRENT_SETTINGS.sync_cycles_low_trq;
        uint8_t interp_max = CRS_CURRENT_SETTINGS.sync_cycles_high_trq;
        if (sid->change == GearChange::_1_2) {
            interp_min += CRS_CURRENT_SETTINGS.sync_cycles_low_trq_adder_1_2;
            interp_max += CRS_CURRENT_SETTINGS.sync_cycles_high_trq_adder_1_2;
        }
        int min_trq = VEHICLE_CONFIG.engine_drag_torque/5.0; // 2x drag torque real
        int max_trq = VEHICLE_CONFIG.engine_drag_torque; // 10x drag torque real
        this->timer_shift = interpolate_float(abs_input_trq,interp_min,interp_max, min_trq, max_trq, InterpType::Linear);

        uint8_t rpm_adder = interpolate_float(sd->input_rpm, &CRS_CURRENT_SETTINGS.sync_cycles_adder_rpm, InterpType::Linear);
        this->timer_shift += rpm_adder;
        this->timer_shift = (float)this->timer_shift * interpolate_float(sid->chars.target_shift_time, &CRS_CURRENT_SETTINGS.sync_multi_shift_speed, InterpType::Linear);

        this->timer_mod = 3;
        this->subphase_shift += 1;
        this->momentum_start_output_rpm = sd->output_rpm;
        this->target_turbine_speed = sd->input_rpm;
        this->momentum_plus_maxtrq = 0;
        this->momentum_plus_maxtrq_filtered = 0;
        if (sid->release_spring_on_clutch < this->p_apply_clutch + this->centrifugal_force_on_clutch) {
            float v = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, this->p_apply_clutch + this->centrifugal_force_on_clutch - sid->release_spring_on_clutch, CoefficientTy::Sliding);
            float trq = 0;
            if (
                (this->upshifting && sd->input_torque > 0) ||
                (!this->upshifting && sd->input_torque < 0)
            ) {
                trq = -abs_input_trq;
            }
            this->momentum_plus_maxtrq = MAX(0, v + trq); // TODO Trq req and adder adapters
        }
    
    }

    int adder = 0;
    if (1 == subphase_shift) {
        int tmp = this->calc_momentum_overlap_2();
        this->momentum_plus_maxtrq = linear_ramp_with_timer(this->momentum_plus_maxtrq, tmp, this->timer_shift);
        this->momentum_plus_maxtrq_filtered = first_order_filter_in_place(80, this->momentum_plus_maxtrq, this->momentum_plus_maxtrq_filtered);
        this->correction_trq = this->calc_correction_trq(this->upshifting ? ShiftStyle::Crossover_Up : ShiftStyle::Crossover_Dn, this->momentum_plus_maxtrq_filtered);

        adder = pm->find_decent_adder_torque(sid->change, this->abs_input_trq, sd->output_rpm);
        this->threshold_rpm = get_rpm_threshold(sid->inf.map_idx, 4);
        if (
            0 == this->timer_shift || 
            sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm || 
            // EGS53 introduced this additional case
            (!this->upshifting && sid->ptr_r_clutch_speeds->off_clutch_speed > CRS_CURRENT_SETTINGS.clutch_stationary_rpm)
        ) {
            // Next phase (No timer, just ends when clutch speed is hit)
            this->subphase_shift += 1;
            if (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm) {
                // Skip phase 2
                this->subphase_shift += 1;
                this->timer_shift = 3;
            }
        }
    } else if (2 == subphase_shift) {
        // Waiting (1)
        this->momentum_plus_maxtrq = this->calc_momentum_overlap_2();
        this->momentum_plus_maxtrq_filtered = first_order_filter_in_place(80, this->momentum_plus_maxtrq, this->momentum_plus_maxtrq_filtered);
        this->correction_trq = this->calc_correction_trq(this->upshifting ? ShiftStyle::Crossover_Up : ShiftStyle::Crossover_Dn, this->momentum_plus_maxtrq_filtered);
        adder = pm->find_decent_adder_torque(sid->change, this->abs_input_trq, sd->output_rpm);
        if (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm) {
            // Next phase
            this->timer_shift = 3;
            this->subphase_shift += 1;
            this->trq_req_up_ramp = true;
            this->trq_req_timer = 6;
        }
    } else if (3 == subphase_shift) {
        adder = this->get_trq_boost_adder();
        int targ_momentum = adder; // boost trq
        if (
            (!upshifting || sd->input_torque < 1) ||
            (upshifting || sd->input_torque > -1)
        ) {
            targ_momentum = (abs_input_trq*2) + adder;
        }

        this->momentum_plus_maxtrq = linear_ramp_with_timer(this->momentum_plus_maxtrq, targ_momentum, this->timer_shift);
        this->momentum_plus_maxtrq_filtered = first_order_filter_in_place(80, this->momentum_plus_maxtrq, this->momentum_plus_maxtrq_filtered);
        this->correction_trq = this->calc_correction_trq(this->upshifting ? ShiftStyle::Crossover_Up : ShiftStyle::Crossover_Dn, this->momentum_plus_maxtrq_filtered);
        if (this->timer_shift == 0 || sid->ptr_r_clutch_speeds->on_clutch_speed < CRS_CURRENT_SETTINGS.clutch_stationary_rpm) {
            this->timer_shift = 3;
            this->subphase_shift += 1;
        }
    } else if (4 == subphase_shift) {
        // Waiting (2)
        adder = this->get_trq_boost_adder();
        if (
            this->timer_shift == 0 ||
            // Additional check by EGS53
            (CRS_CURRENT_SETTINGS.clutch_stationary_rpm > -sid->ptr_r_clutch_speeds->on_clutch_speed)
        ) {
            sid->tcc->shift_end();
            ret = PHASE_MAX_PRESSURE;
        }
    }
    // Trq adder 2/3 are included in trq_adder for this step
    this->trq_adder = this->trq_adder_1;
    uint16_t torque = abs_input_trq + this->trq_adder + this->correction_trq;
    uint16_t targ = MAX(
        this->set_p_apply_clutch_with_spring(pm->p_clutch_with_coef(sid->targ_g, sid->applying, torque, CoefficientTy::Sliding)), 
        this->set_p_apply_clutch_with_spring(this->p_apply_overlap_begin)
    );
    this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);

    this->shift_sol_pressure = this->correct_shift_shift_pressure(this->p_apply_clutch);
    // Calculations for MOD pressure
    uint16_t targ_pmod = this->calc_overlap2_mod();
    this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, targ_pmod, this->timer_mod);
    return ret;
}

uint16_t CrossoverShift::calc_overlap_mod() {
    int16_t p_mod = 0;
    if (abs_input_trq > this->trq_at_apply_clutch) {
        this->trq_at_release_clutch = abs_input_trq - this->trq_at_apply_clutch;
        p_mod = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, this->trq_at_release_clutch, CoefficientTy::Release);
    }
    if (p_mod + sid->release_spring_off_clutch < this->centrifugal_force_off_clutch) {
        p_mod = 0;
        this->trq_at_release_clutch = 0;
    } else {
        uint16_t t = MAX(0, p_mod + sid->release_spring_off_clutch - this->centrifugal_force_off_clutch);
        p_mod = (uint16_t)((float)t*sid->inf.centrifugal_factor_off_clutch);
    }
    return this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p_mod);
}

uint16_t CrossoverShift::calc_overlap_mod_min(int p_shift) {
    int p_mod = MAX(0, sid->release_spring_off_clutch - this->centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch;
    return this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p_mod);
}

uint16_t CrossoverShift::calc_overlap2_mod() {
    int p_shift = (int)this->p_apply_clutch * sid->inf.pressure_multi_spc_int;
    p_shift /= 1000;
    int centrifugal = this->centrifugal_force_off_clutch * sid->inf.pressure_multi_mpc_int * sid->inf.centrifugal_factor_off_clutch;
    centrifugal /= 1000;
    
    int base = 200 * sid->inf.pressure_multi_mpc_int;
    base /= 1000;

    centrifugal += base;

    if (centrifugal < p_shift) {
        centrifugal = p_shift - centrifugal;
    } else {
        centrifugal = 0;
    }
    int p_mod = centrifugal + sid->inf.mpc_pressure_spring_reduction;
    p_mod = MIN(MAX(p_mod, 0), sid->MOD_MAX);
    return p_mod;
}   


float ramping_mod_multi[8] = { 0.25, 0.20, 1.0, 1.0, 1.0, 1.0, 1.0, 0.1 };
uint16_t CrossoverShift::fill_ramping_mod_p() {
    float p_shift = this->p_apply_clutch * sid->inf.pressure_multi_spc_int;
    p_shift /= 1000;
    int p_centrifugal = this->centrifugal_force_off_clutch * sid->inf.pressure_multi_mpc_int * ramping_mod_multi[sid->inf.map_idx];
    p_centrifugal /= 1000;
    return MAX(0.0, MIN(sid->MOD_MAX, p_shift - p_centrifugal + sid->inf.mpc_pressure_spring_reduction));
}

uint16_t CrossoverShift::max_p_mod_pressure() {
    return pm->find_working_mpc_pressure(sid->targ_g);
}

int16_t CrossoverShift::calc_momentum_overlap_2() {
    int ret = this->get_trq_adder_map_val(); //+ this->torque_req_val; // TODO + adapters
    if (
        // WTF. Just mimic original logic, but not sure how this works
        (!this->upshifting || sd->input_torque < 1) &&
        (this->upshifting || sd->input_torque > -1)
    ) {
        ret += (abs_input_trq*2);
    }
    return MAX(0, ret);
}