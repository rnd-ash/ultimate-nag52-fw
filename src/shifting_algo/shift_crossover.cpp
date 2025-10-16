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

void CrossoverShift::calc_shift_flags(uint32_t* dest) {
    *dest = 0;
    if (sd->pedal_pos < 10) {
        if ((sid->targ_g < sid->curr_g) && (sid->targ_g == GearboxGear::Third || sid->targ_g == GearboxGear::Fourth)) {
            *dest |= SHIFT_FLAG_COAST_54_43;
        }
        *dest |= SHIFT_FLAG_COAST;
    }
    if (sid->change == GearChange::_1_2 || sid->change == GearChange::_3_2) {
        *dest |= SHIFT_FLAG_FREEWHEELING;
    }
}

uint16_t CrossoverShift::high_fill_pressure() {
    return sid->prefill_info.fill_pressure_on_clutch;
}

uint8_t FAC_TABLE[8] = {90, 90, 85, 70, 100, 100, 100, 100};
uint16_t get_rpm_threshold(uint8_t shift_idx, uint16_t abs_trq, uint8_t ramp_cycles) {
    int bvar1 = 6;
    float inertia = ShiftHelpers::get_shift_intertia(shift_idx);
    float res = ((abs_trq*5) * (ramp_cycles+(2*bvar1))) / inertia;
    res *= (MECH_PTR->turbine_drag[shift_idx]/10.0);
    res *= (float)FAC_TABLE[shift_idx] / 100.0;
    if (res < 130) {
        res = 130;
    }
    return (uint16_t)res;
}



uint8_t CrossoverShift::step_internal(
    bool stationary,
    bool is_upshift
) {
    uint8_t ret = STEP_RES_CONTINUE;
    if (phase_id == PHASE_BLEED) {
        ret = this->phase_bleed(pm, is_upshift);
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

    return ret;
}


uint8_t CrossoverShift::phase_fill() {
    uint8_t ret = STEP_RES_CONTINUE;
    uint16_t high_filling_p = sid->prefill_info.fill_pressure_on_clutch;
    uint16_t low_filling_p = sid->prefill_info.low_fill_pressure_on_clutch;
    if (0 == this->subphase_shift) {
        // Set vars
        this->timer_shift = sid->prefill_info.fill_time/20;
        this->subphase_shift += 1;
    }
    if (1 == this->subphase_shift) {
        // High filling
        this->p_apply_clutch = set_p_apply_clutch_with_spring(high_filling_p);
        if (0 == this->timer_shift) {
            this->timer_shift = 3; // FILL_CAL->fill_ramp_time
            // Roughly drag torque
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
    this->shift_sol_pressure = pm->correct_shift_shift_pressure(sid->inf.map_idx, this->p_apply_clutch + spc_p_offset);

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
    this->max_trq_apply_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_apply_clutch, CoefficientTy::Sliding);
    this->trq_adder = pm->find_decent_adder_torque(sid->change, this->abs_input_trq, sd->output_rpm);
    if (0 == subphase_shift) {
        this->p_apply_overlap_begin = this->p_apply_clutch;
        uint8_t interp_min = 5; // RELEASE_CAL->0x24
        uint8_t interp_max = 3; // RELEASE_CAL->0x25
        if (sid->change == GearChange::_1_2) {
            interp_min += 1; // RELEASE_CAL->0x46
            interp_max += 0; // RELEASE_CAL->0x47
        }
        this->timer_shift = interpolate_float(abs_input_trq,interp_min,interp_max,80,400, InterpType::Linear);

        uint8_t rpm_adder = interpolate_float(sd->input_rpm,0,0,1000,4000, InterpType::Linear);
        this->timer_shift += rpm_adder;
        this->trq_req_timer = this->timer_shift;
        this->subphase_shift += 1;
    }
    if (1 == subphase_shift) {
        uint16_t c_trq_apply = pm->p_clutch_with_coef(
            sid->targ_g,
            sid->applying,
            abs_input_trq + this->trq_adder,
            CoefficientTy::Sliding
        );
        uint16_t targ = MAX(this->set_p_apply_clutch_with_spring(c_trq_apply), this->p_apply_overlap_begin);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        uint16_t p_mod_1 = this->fun_0d86b4();
        uint16_t p_mod_2 = this->fun_0d8a10(this->p_apply_overlap_begin);
        this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
    }
    this->shift_sol_pressure = pm->correct_shift_shift_pressure(sid->inf.map_idx, this->p_apply_clutch + spc_p_offset);


    if (this->timer_shift == 0 || sid->ptr_r_clutch_speeds->on_clutch_speed < 130 || sid->ptr_r_clutch_speeds->off_clutch_speed > 130) {
        // Next phase on clutch movement or timeout
        ret = PHASE_OVERLAP2;
    }

    return ret;
}

uint8_t CrossoverShift::phase_overlap2() {
    uint8_t ret = STEP_RES_CONTINUE;
    this->trq_adder = pm->find_decent_adder_torque(sid->change, this->abs_input_trq, sd->output_rpm);
    this->max_trq_apply_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_apply_clutch, CoefficientTy::Sliding);
    
    if (0 == subphase_shift) {
        this->p_apply_overlap_begin = this->p_apply_clutch;
        uint8_t interp_min = 5; // RELEASE_CAL->0x24
        uint8_t interp_max = 3; // RELEASE_CAL->0x25
        if (sid->change == GearChange::_1_2) {
            interp_min += 1; // RELEASE_CAL->0x46
            interp_min += 0; // RELEASE_CAL->0x47
        }
        this->timer_shift = interpolate_float(abs_input_trq,interp_min,interp_max, 80, 400, InterpType::Linear);

        uint8_t rpm_adder = interpolate_float(sd->input_rpm,0,0,1000,4000, InterpType::Linear);
        this->timer_shift += rpm_adder;

        this->timer_mod = 0; // 0x69
        if (sd->converted_torque < 0) {
            this->timer_mod = (float)this->timer_mod * 1.0;
        }
        this->subphase_shift += 1;
        
        this->momentum_start_output_rpm = sd->output_rpm;
        this->momentum_start_turbine_rpm = sd->input_rpm;
        this->momentum_plus_maxtrq = sd->indicated_torque;
        this->momentum_plus_maxtrq_1 = 0;
    
    }
    if (1 == subphase_shift) {
        this->trq_adder_2 += 8;

        uint16_t torque = abs_input_trq + this->trq_adder + this->trq_req_val + this->trq_adder_2;
        uint16_t targ = pm->p_clutch_with_coef(sid->targ_g, sid->applying, torque, CoefficientTy::Sliding);
        targ = MAX(targ, this->p_apply_overlap_begin);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        uint16_t rpm_targ = get_rpm_threshold(sid->inf.map_idx, abs_input_trq, 4);
        if (0 == this->timer_shift || sid->ptr_r_clutch_speeds->on_clutch_speed < rpm_targ) {
            // Next phhase
            this->subphase_shift += 1;
        }
    } else if (2 == subphase_shift) {
        this->trq_adder_2 += 4;
        uint16_t torque = abs_input_trq + this->trq_adder + this->trq_req_val + this->trq_adder_2;
        this->p_apply_clutch = pm->p_clutch_with_coef(sid->targ_g, sid->applying, torque, CoefficientTy::Sliding);
        // Keep ramping until we hit target speed
        if (sid->ptr_r_clutch_speeds->on_clutch_speed < 130) {
            // Next phhase
            this->timer_shift = 3;
            this->subphase_shift += 1;
        }
    } else if (3 == subphase_shift) {
        this->trq_adder_2 += 2;
        uint16_t torque = abs_input_trq + this->trq_adder + this->trq_req_val + this->trq_adder_2;
        this->p_apply_clutch = pm->p_clutch_with_coef(sid->targ_g, sid->applying, torque, CoefficientTy::Sliding);
        if (this->timer_shift == 0) {
            this->trq_req_up_ramp = true;
            ret = PHASE_MAX_PRESSURE;
        }
    }
    this->shift_sol_pressure = pm->correct_shift_shift_pressure(sid->inf.map_idx, this->p_apply_clutch + spc_p_offset);
    // Calculations for MOD pressure
    uint16_t pmod = this->fun_0d8a66();
    this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, pmod, this->timer_mod);
    return ret;
}

uint16_t CrossoverShift::fun_0d86b4() {
    uint16_t p_mod = 0;
    if (abs_input_trq > this->max_trq_apply_clutch) {
        uint16_t d = abs_input_trq - this->max_trq_apply_clutch;
        p_mod = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, d, CoefficientTy::Release);
    }
    if (p_mod + sid->release_spring_off_clutch < this->centrifugal_force_off_clutch) {
        p_mod = 0;
    } else {
        uint16_t t = p_mod + sid->release_spring_off_clutch - this->centrifugal_force_off_clutch;
        p_mod = (uint16_t)((float)t*sid->inf.centrifugal_factor_off_clutch);
    }
    return this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p_mod);
}

uint16_t CrossoverShift::fun_0d8a10(uint16_t p_shift) {
    uint16_t p_mod = 0;
    if (sid->release_spring_off_clutch > this->centrifugal_force_off_clutch) {
        p_mod = (uint16_t)((float)(sid->release_spring_off_clutch - this->centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch);
    }
    return this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p_mod);
}

uint16_t CrossoverShift::fun_0d8a66() {
    float p_shift = this->p_apply_clutch * sid->inf.pressure_multi_spc;
    float centrifugal = (
        (float)this->centrifugal_force_off_clutch * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch
    );
    float holding = sid->release_spring_off_clutch * sid->inf.pressure_multi_mpc;
    int16_t p_mod = p_shift - centrifugal + holding;
    p_mod += sid->inf.mpc_pressure_spring_reduction;
    p_mod = MIN(MAX(p_mod, 0), sid->MOD_MAX);
    return p_mod;
}   


float ramping_mod_multi[8] = { 0.25, 0.20, 1.0, 1.0, 1.0, 1.0, 1.0, 0.1 };
uint16_t CrossoverShift::fill_ramping_mod_p() {
    float p_shift = (float)this->p_apply_clutch * sid->inf.pressure_multi_spc;
    float p_centrifugal = (float)this->centrifugal_force_off_clutch * sid->inf.pressure_multi_mpc * ramping_mod_multi[sid->inf.map_idx];
    return MAX(0.0, MIN(sid->MOD_MAX, p_shift - p_centrifugal + sid->inf.mpc_pressure_spring_reduction));
}

uint16_t CrossoverShift::max_p_mod_pressure() {
    return pm->find_working_mpc_pressure(sid->targ_g);
}