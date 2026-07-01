#include "shift_crossover.h"
#include <egs_calibration/calibration_structs.h>
#include <math.h>

const DRAM_ATTR uint8_t PHASE_BLEED            = 0;
const DRAM_ATTR uint8_t PHASE_FILL             = 1;
const DRAM_ATTR uint8_t PHASE_OVERLAP          = 2;
const DRAM_ATTR uint8_t PHASE_OVERLAP2         = 3;
const DRAM_ATTR uint8_t PHASE_MAX_PRESSURE     = 4;
const DRAM_ATTR uint8_t PHASE_END_CONTROL      = 5;

uint8_t FAC_TABLE[8] = {90, 90, 85, 70, 100, 100, 100, 100};
float RAMP_LIMITS[8] = {0.325, 0.625, 0.55, 0, 0, 0, 0, 0};
float ADAPT_LIMITS[8] = {1.0, 1.25, 1.75, 2.75, 0, 0, 0.75, 0.75};

CrossoverShift::CrossoverShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {
    this->adapting_trq_limit = ((float)VEHICLE_CONFIG.engine_drag_torque/10.0)*ADAPT_LIMITS[sid->inf.map_idx];
    this->ramp_filling_trq_limit = ((float)VEHICLE_CONFIG.engine_drag_torque/10.0)*RAMP_LIMITS[sid->inf.map_idx];
}

CrossoverShift::~CrossoverShift() {
}

uint8_t CrossoverShift::max_shift_stage_id() {
    return PHASE_END_CONTROL;
}

// P1 - IDX
// P2 - Cycles
uint16_t CrossoverShift::get_rpm_threshold(uint8_t shift_idx, uint8_t ramp_cycles) {
    float torque = this->get_trq_adder_map_val() + this->get_trq_boost_adder() + this->torque_req_val;
    float bVar1 = 4;
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
        this->fill_adapt();
    } else if (phase_id == PHASE_OVERLAP) {
        ret = this->phase_overlap();
        this->overlap_adapt();
    } else if (phase_id == PHASE_OVERLAP2) {
        ret = this->phase_overlap2();
        this->overlap2_adapt();
    } else if (phase_id == PHASE_MAX_PRESSURE) {
        ret = this->phase_maxp(sd);
    } else if (phase_id == PHASE_END_CONTROL) {
        ret = this->phase_end_ctrl();
    } else {
        ret = STEP_RES_END_SHIFT; // WTF? Should never happen
    }

    if (phase_id >= PHASE_FILL && this->do_fill_time_adaptation && !this->end_of_fill_time_adapt) {
        this->fill_time_adapt_timer += 1;
    }

    // Do torque request stuff here
    this->torque_req_out = 0;
    if (sd->indicated_torque > sd->min_torque && sd->engine_rpm > 1100) {
        int intervension_out = 0;
        this->trq_req_compensate_val = 0;
        // Enable Trq req if 1/2 Drag torque or higher
        if (sd->indicated_torque > VEHICLE_CONFIG.engine_drag_torque/20.0) {
            float trq_max = VEHICLE_CONFIG.engine_drag_torque*2; // 20x drag torque
            float min_rpm_input = 1000; // Approx
            float max_rpm_input = VEHICLE_CONFIG.engine_type == 0 ? 4500 : 6000;

            float multi_engine_trq;
            float multi_rpm;
            multi_engine_trq = interpolate_float(sd->indicated_torque, 0.0, 0.3, 0, trq_max, InterpType::Linear);
            multi_engine_trq *= interpolate_float(sid->chars.target_shift_time, 1.0, 2.0, 1000, 100, InterpType::Linear);
            multi_rpm = interpolate_float(sd->input_rpm, 1.0, 1.25, min_rpm_input, max_rpm_input, InterpType::Linear);

            float out = (float)sd->indicated_torque * (multi_engine_trq*multi_rpm);
            intervension_out = MIN(out, (float)sd->indicated_torque*0.9);

            // Compensate
            if (intervension_out != 0) {
                if (this->correction_trq + 0 <= 0) {
                    int min = (40 * (this->correction_trq + 0)) / 100;
                    this->trq_req_compensate_val = MAX(min, -intervension_out);
                } else {
                    this->trq_req_compensate_val = MAX(0, ((55 * sd->indicated_torque) / 100) - intervension_out);
                    int offset = 0;
                    if (sid->adaptation_mgr) {
                        offset = sid->adaptation_mgr->get_applying_torque_offset(sid->inf.map_idx);
                    }
                    this->trq_req_compensate_val = MIN(this->trq_req_compensate_val, (50 * (this->correction_trq + offset))/100);
                }
            }
        }
        intervension_out = MAX(0, intervension_out + this->trq_req_compensate_val);
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
    if (sid->trq_req_en) {
        this->emergency_trq_val = 0;
    } else {
        this->emergency_trq_val = (this->torque_req_out * pm->release_coefficient()) / 100;
    }

    // Output to CAN
    if (0 != torque_req_out && sid->trq_req_en) {
        torque_req_out = MIN(torque_req_out, sd->indicated_torque);
        sid->ptr_w_trq_req->amount = MAX(0, sd->indicated_torque - torque_req_out);
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
        this->cycles_high_filling = sid->prefill_info.fill_cycles;
        if (this->sid->adaptation_mgr) {
            int8_t offset = sid->adaptation_mgr->get_prefill_cycles_offset(sid->inf.map_idx);
            if (((int16_t)(this->cycles_high_filling) + offset) > 0) {
                this->cycles_high_filling += offset;
            } else {
                this->cycles_high_filling = 0;
            }
        }
        this->timer_shift = this->cycles_high_filling;
        this->subphase_shift += 1;
        this->timer_emergency = -1;
    }
    if (1 == this->subphase_shift) {
        // High filling
        this->p_apply_clutch = set_p_apply_clutch_with_spring(high_filling_p);
        if (0 == this->timer_shift) {
            if (
                abs_input_trq < this->ramp_filling_trq_limit && upshifting
            ) {
                // Ramp filling
                this->subphase_shift = 4;
                // Switch adapting to pressure
                this->do_fill_time_adaptation = false;
                this->do_fill_pressure_adaptation = false;
                this->fill_via_ramp = true;
            } else {
                // Non ramp filling
                this->subphase_shift = 2;
                this->fill_via_ramp = false;
                this->cycles_ramp_to_low_filling = 3;
                if (race == sid->profile) {
                    this->cycles_ramp_to_low_filling = 1;
                }
                this->cycles_low_filling = 5;
                this->timer_shift = cycles_ramp_to_low_filling;
            }
        }
        uint16_t p_mod_1 = this->calc_mod_with_filling_trq_and_freewheeling(this->p_apply_clutch);
        uint16_t p_mod_2 = this->calc_mod_min_abs_trq(low_filling_p);
        this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
        if (this->do_fill_pressure_adaptation || (this->fill_via_ramp && this->upshifting)) {
            sid->tcc->shift_start(this->upshifting, false, true);
        }
    } 
    else if (2 == this->subphase_shift) {
        // Ramp to low filling P
        uint16_t targ = this->set_p_apply_clutch_with_spring(low_filling_p);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        if (0 == this->timer_shift) {
            this->timer_shift = this->cycles_low_filling;
            this->subphase_shift += 1;
        }

        if ((abs_input_trq < this->adapting_trq_limit && this->do_fill_time_adaptation) || ((sid->shift_flags & SHIFT_FLAG_COAST_54_43) != 0)) {
            this->mod_sol_pressure = calc_overlap2_mod();
        } else {
            this->do_fill_time_adaptation = false;
            uint16_t p_mod_1 = this->calc_mod_with_filling_trq_and_freewheeling(this->p_apply_clutch);
            uint16_t p_mod_2 = this->calc_mod_min_abs_trq(low_filling_p);
            this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
        }
    } else if (3 == this->subphase_shift) {
        // Low filling P
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(low_filling_p);
        if ((abs_input_trq < this->adapting_trq_limit && this->do_fill_time_adaptation) || ((sid->shift_flags & SHIFT_FLAG_COAST_54_43) != 0)) {
            this->mod_sol_pressure = calc_overlap2_mod();
        } else {
            this->do_fill_time_adaptation = false;
            uint16_t p_mod_1 = this->calc_mod_with_filling_trq_and_freewheeling(this->p_apply_clutch);
            uint16_t p_mod_2 = this->calc_mod_min_abs_trq(low_filling_p);
            this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
        }
        if (0 == this->timer_shift) {
            ret = PHASE_OVERLAP;
        }
    }
    else if (4 == this->subphase_shift) {
        // Drop to 0 mBar for adapting to start (Pre ramping)
        this->p_apply_clutch = this->set_p_apply_clutch_with_spring(0);
        this->mod_sol_pressure = this->calc_overlap2_mod();
        this->timer_shift = 25;
        this->subphase_shift += 1;
    } else if (5 == this->subphase_shift) {
        // Low filling test
        uint16_t targ = this->set_p_apply_clutch_with_spring(500);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        this->mod_sol_pressure = this->calc_overlap2_mod();
        if (0 == this->timer_shift || sid->ptr_r_clutch_speeds->off_clutch_speed > CRS_CURRENT_SETTINGS.clutch_stationary_rpm) {
            this->timer_shift = 12;
            this->subphase_shift += 1;
        }
    } else if (6 == this->subphase_shift) {
        // Higher filling test
        uint16_t targ = this->set_p_apply_clutch_with_spring(700);
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
        this->mod_sol_pressure = this->calc_overlap2_mod();
        if (0 == this->timer_shift || sid->ptr_r_clutch_speeds->off_clutch_speed > CRS_CURRENT_SETTINGS.clutch_stationary_rpm) {
            ret = PHASE_OVERLAP;
        }
    }

    if (this->subphase_shift < 4) {
        // normal filling exit check
        if (
            sid->ptr_r_clutch_speeds->off_clutch_speed > CRS_CURRENT_SETTINGS.clutch_stationary_rpm &&
            !this->torque_jumped &&
            (
                (
                    this->upshifting &&
                    sd->input_torque > VEHICLE_CONFIG.engine_drag_torque/10.0
                ) ||
                (
                    !this->upshifting &&
                    sd->input_torque < -VEHICLE_CONFIG.engine_drag_torque/10.0
                )
            )
        )  {
            ret = PHASE_OVERLAP;
        }
    } else {
        // Ramp filling exit check
        if (sid->ptr_r_clutch_speeds->on_clutch_speed <= CRS_CURRENT_SETTINGS.clutch_stationary_rpm || sd->input_torque > this->ramp_filling_trq_limit*1.5) {
            ret = PHASE_OVERLAP;
        }
    }

    // Write Shift sol pressure
    this->shift_sol_pressure = this->correct_shift_shift_pressure(this->p_apply_clutch);
    return ret;
}

uint8_t CrossoverShift::phase_overlap() {
    uint8_t ret = STEP_RES_CONTINUE;
    this->trq_at_apply_clutch = this->calc_max_trq_on_clutch(this->p_apply_clutch, CoefficientTy::Sliding);

    if (0 == subphase_shift) {
        this->calculate_accel_trq_corr();
        this->trq_adder = 0;
        this->timer_emergency = 5000/20; // 5 seconds for timeout for overlap
        this->p_apply_overlap_begin = this->p_apply_clutch + centrifugal_force_on_clutch;
        this->timer_shift = ShiftHelpers::cycles_crossover_overlap(sid->change, abs_input_trq);

        uint8_t rpm_adder = interpolate_float(sd->input_rpm, &CRS_CURRENT_SETTINGS.overlap_cycles_adder_rpm, InterpType::Linear);
        this->timer_shift += rpm_adder;
        this->subphase_shift += 1;
        this->trq_req_timer = MAX(3, this->timer_shift);
        this->trq_req_down_ramp = true;
    }
    if (!this->upshifting) {
        this->calculate_accel_trq_corr();
    }

    if (sid->adaptation_mgr) {
        this->trq_adder = sid->adaptation_mgr->get_applying_torque_offset(sid->inf.map_idx);
    }
    this->trq_adder -= this->calculate_dynamic_inertia();
    this->trq_adder -= this->trq_req_compensate_val;

    uint16_t c_trq_apply = pm->p_clutch_with_coef_signed(
        sid->targ_g,
        sid->applying,
        MAX(0, (int)abs_input_trq + this->trq_adder),
        CoefficientTy::Sliding
    );

    uint16_t targ = MAX(
        this->set_p_apply_clutch_with_spring(c_trq_apply),
        this->p_apply_overlap_begin - this->centrifugal_force_on_clutch
    );
    this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);

    // Mod pressure depends on the current situation
    if (abs_input_trq < this->ramp_filling_trq_limit*1.5 && fill_via_ramp) {
        this->mod_sol_pressure = this->calc_overlap2_mod();
    } else if (abs_input_trq < this->adapting_trq_limit && (this->do_fill_time_adaptation || (sid->shift_flags & SHIFT_FLAG_COAST_54_43) != 0)) {
        this->fill_via_ramp = false;
        this->mod_sol_pressure = this->calc_overlap2_mod();
    } else {
        this->fill_via_ramp = false;
        this->do_fill_time_adaptation = false;
        uint16_t p_mod_1 = this->calc_overlap_mod();
        uint16_t p_mod_2 = this->calc_overlap_mod_min(MAX(targ, this->p_apply_overlap_begin));
        this->mod_sol_pressure = MAX(p_mod_1, p_mod_2);
    }
    if (
        0 == this->timer_shift ||
        sid->ptr_r_clutch_speeds->off_clutch_speed > CRS_CURRENT_SETTINGS.clutch_stationary_rpm
    ) {
        // Next phase on clutch movement or timeout
        sid->tcc->shift_start(this->upshifting, false, false);
        ret = PHASE_OVERLAP2;
    }
    this->shift_sol_pressure = this->correct_shift_shift_pressure(this->p_apply_clutch);
    return ret;
}

uint16_t CrossoverShift::get_trq_adder_map_val() {
    float map_val = pm->find_decent_adder_torque(sid->change, this->abs_input_trq, sd->output_rpm);
    if (upshifting) {
        if (sid->profile == manual) {
            map_val *= 1.25;
        } else if (sid->profile == race) {
            map_val *= 2.0;
        }
    } else {
        // So dynamic downshifts feel nicer
        if (sid->manual_shift) {
            map_val *= 1.25;
        }
        if (sid->profile == manual) {
            map_val *= 1.1;
        } else if (sid->profile == race) {
            map_val *= 1.25;
        }
    }
    // Correct based on acceleration (Ordering is important here)
    if (upshifting) {
        map_val = MAX(0, map_val - this->torque_accel_corr);
        if (sd->input_torque < 0) {
            map_val = MAX(0, map_val - (2*abs_input_trq));
        }
    } else {
        if (sd->input_torque > 0) {
            map_val = MAX(0, map_val - (2*abs_input_trq));
        }
        map_val = MAX(0, map_val + this->torque_accel_corr);
    }
    return map_val;
}

void CrossoverShift::calculate_accel_trq_corr() {
    if (VEHICLE_CONFIG.wheel_circumference != 0) {
        int accel_corr_trq = 0;
        accel_corr_trq = ((ShiftHelpers::get_shift_intertia(sid->inf.map_idx) * MECH_PTR->ratio_table[sid->inf.curr_g] / 1000) * 6.0 * sid->diff_ratio/10.0) * sd->acceleration_ms2;
        accel_corr_trq /= (int)VEHICLE_CONFIG.wheel_circumference;
        accel_corr_trq /= 10;
        this->torque_accel_corr = MIN(accel_corr_trq, this->torque_accel_corr);
    }
}

uint16_t CrossoverShift::get_trq_boost_adder() {
    uint16_t ret = 0;
    uint16_t map_val = this->get_trq_adder_map_val();
    int min = VEHICLE_CONFIG.engine_drag_torque/11.0; // ~0.9 drag torque
    float boost_trq_adder = ((pm->sliding_coefficient()/pm->release_coefficient())-1.0) * (float)abs_input_trq;
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

    this->threshold_rpm = get_rpm_threshold(sid->inf.map_idx, 3);
    this->calculate_accel_trq_corr();
    if (!this->trq_req_down_ramp && abs(sid->ptr_r_clutch_speeds->off_clutch_speed) >= CRS_CURRENT_SETTINGS.clutch_stationary_rpm) {
        this->trq_req_timer = 3;
        this->trq_req_down_ramp = true;
    }
    int adaptation_adder = 0;
    if (sid->adaptation_mgr) {
        adaptation_adder = sid->adaptation_mgr->get_applying_torque_offset(sid->inf.map_idx);
    }


    if (0 == subphase_shift) {
        this->timer_emergency = 5000/20; // 5 seconds for timeout for overlap2
        this->timer_shift = ShiftHelpers::cycles_crossover_overlap2(sid->change, abs_input_trq);

        uint8_t rpm_adder = interpolate_float(sd->input_rpm, &CRS_CURRENT_SETTINGS.sync_cycles_adder_rpm, InterpType::Linear);
        this->timer_shift += rpm_adder;
        this->timer_shift = (float)this->timer_shift * interpolate_float(sid->chars.target_shift_time, &CRS_CURRENT_SETTINGS.sync_multi_shift_speed, InterpType::Linear);

        this->timer_mod = 3;
        this->subphase_shift += 1;
        this->momentum_start_output_rpm = sd->output_rpm;
        this->target_turbine_speed = sd->input_rpm;
        this->momentum_ctrl = 0;
        this->momentum_ctrl_filtered = 0;

        float v = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, MAX(0, this->p_apply_clutch + this->centrifugal_force_on_clutch - sid->release_spring_on_clutch), CoefficientTy::Sliding);
        float reduction = 0;
        if (
                (this->upshifting && sd->input_torque > 0) ||
                (!this->upshifting && sd->input_torque < 0)
            ) {
                reduction = abs_input_trq;
            } else {
                // Inverted so this leads to a + abs_input_trq in the Equation below
                reduction = -abs_input_trq;
            }
            this->momentum_ctrl = MAX(0, v + this->trq_req_compensate_val - reduction); // TODO Trq req and adder adapters
    }

    if (1 == subphase_shift) {
        this->trq_adder = this->get_trq_adder_map_val() + adaptation_adder + this->emergency_trq_val - this->trq_req_compensate_val;
        int tmp = this->calc_momentum_overlap_2();
        this->momentum_ctrl = linear_ramp_with_timer(this->momentum_ctrl, tmp, this->timer_shift);
        this->momentum_ctrl_filtered = linear_interp_with_percentage(80, this->momentum_ctrl, this->momentum_ctrl_filtered);
        this->correction_trq = this->calc_correction_trq(this->upshifting ? ShiftStyle::Crossover_Up : ShiftStyle::Crossover_Dn, this->momentum_ctrl_filtered);

        if (
            0 == this->timer_shift ||
            sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm
        ) {
            // Next phase (No timer, just ends when clutch speed is hit)
            this->subphase_shift += 1;
            if ((sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm)) {
                // Skip phase 2
                this->subphase_shift += 1;
                this->timer_shift = 3;
            }
        }
    } else if (2 == subphase_shift) {
        // Waiting (1)
        this->trq_adder = this->get_trq_adder_map_val() + adaptation_adder + this->emergency_trq_val - this->trq_req_compensate_val;
        this->momentum_ctrl = this->calc_momentum_overlap_2();
        this->momentum_ctrl_filtered = linear_interp_with_percentage(80, this->momentum_ctrl, this->momentum_ctrl_filtered);
        this->correction_trq = this->calc_correction_trq(this->upshifting ? ShiftStyle::Crossover_Up : ShiftStyle::Crossover_Dn, this->momentum_ctrl_filtered);
        if (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm) {
            // Next phase
            this->timer_shift = 3;
            this->trq_req_timer = 6;
            this->subphase_shift += 1;
        }
    } else if (3 == subphase_shift) {
        this->trq_adder = this->get_trq_boost_adder() + adaptation_adder - this->trq_req_compensate_val;
        int targ_momentum = this->get_trq_boost_adder();
        if (
            (upshifting && sd->input_torque <= 0) ||
            (!upshifting && sd->input_torque >= 0)
        ) {
            targ_momentum += (abs_input_trq*2);
        }

        this->momentum_ctrl = linear_ramp_with_timer(this->momentum_ctrl, targ_momentum, this->timer_shift);
        this->momentum_ctrl_filtered = linear_interp_with_percentage(80, this->momentum_ctrl, this->momentum_ctrl_filtered);
        this->correction_trq = this->calc_correction_trq(this->upshifting ? ShiftStyle::Crossover_Up : ShiftStyle::Crossover_Dn, this->momentum_ctrl_filtered);
        if (this->timer_shift == 0 || sid->ptr_r_clutch_speeds->on_clutch_speed < CRS_CURRENT_SETTINGS.clutch_stationary_rpm) {
            this->timer_shift = 3;
            this->subphase_shift += 1;
            sid->tcc->shift_end();
        }
    } else if (4 == subphase_shift) {
        // Waiting (2)
        this->trq_adder = this->get_trq_boost_adder() + adaptation_adder - this->trq_req_compensate_val;
        if (
            this->timer_shift == 0
        ) {
            // Analyze adaptations
            ESP_LOGI("ADAPT", "End adaptation flags: %d %d %d", do_fill_time_adaptation, do_fill_pressure_adaptation, do_torque_adaptation);
            ret = PHASE_MAX_PRESSURE;
        }
    }

    if (!this->trq_req_up_ramp && sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm && subphase_shift >= 2) {
        this->trq_req_timer = 3;
        this->trq_req_up_ramp = true;
    }

    int torque = MAX(0, (int)abs_input_trq + this->trq_adder + this->correction_trq);
    uint16_t targ = MAX(
        this->set_p_apply_clutch_with_spring(pm->p_clutch_with_coef_signed(sid->targ_g, sid->applying, torque, CoefficientTy::Sliding)), 
        this->p_apply_overlap_begin - centrifugal_force_on_clutch
    );
    if (1 == subphase_shift || 3 == subphase_shift) {
        this->p_apply_clutch = linear_ramp_with_timer(this->p_apply_clutch, targ, this->timer_shift);
    } else {
        this->p_apply_clutch = targ;
    }

    this->shift_sol_pressure = this->correct_shift_shift_pressure(this->p_apply_clutch);
    // Calculations for MOD pressure
    uint16_t targ_pmod = this->calc_overlap2_mod();
    this->mod_sol_pressure = linear_ramp_with_timer(this->mod_sol_pressure, targ_pmod, this->timer_mod);
    return ret;
}

uint16_t CrossoverShift::calc_overlap_mod() {
    int p_mod = 0;
    if (abs_input_trq > this->trq_at_apply_clutch) {
        this->trq_at_release_clutch = abs_input_trq - this->trq_at_apply_clutch;
        p_mod = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, this->trq_at_release_clutch, CoefficientTy::Release);
    }
    p_mod = MAX(0, p_mod + sid->release_spring_off_clutch - this->centrifugal_force_off_clutch);
    p_mod *= sid->inf.centrifugal_factor_off_clutch_int;
    p_mod /= 100;
    return this->calc_mpc_sol_shift_ps(this->p_apply_clutch, p_mod);
}

uint16_t CrossoverShift::calc_overlap_mod_min(int p_shift) {
    int p_mod = MAX(0, sid->release_spring_off_clutch - this->centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch_int;
    p_mod /= 100; // centrifugal factor
    return this->calc_mpc_sol_shift_ps(p_shift, p_mod);
}

uint16_t CrossoverShift::calc_overlap2_mod() {
    int p_shift = (int)this->p_apply_clutch * sid->inf.pressure_multi_spc_int;
    p_shift /= 1000;
    int centrifugal = this->centrifugal_force_off_clutch * sid->inf.pressure_multi_mpc_int * sid->inf.centrifugal_factor_off_clutch_int;
    centrifugal /= 100; // centrifugal factor
    centrifugal /= 1000;
    
    int base = 250 * sid->inf.pressure_multi_mpc_int;
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
    int p_shift = pm->p_clutch_with_coef(sid->targ_g, sid->applying, abs_input_trq, CoefficientTy::Release);

    p_shift = MIN(sid->SPC_MAX, MAX(0, p_shift + sid->release_spring_on_clutch - centrifugal_force_on_clutch));

    p_shift *= sid->inf.pressure_multi_spc_int;
    p_shift /= 1000;

    int p_mod = this->centrifugal_force_off_clutch * sid->inf.pressure_multi_mpc_int;
    p_mod /= 1000;
    p_mod *= sid->inf.centrifugal_factor_off_clutch_int;
    p_mod /= 100; // centrifugal factor

    return MAX(
        0,
        p_shift - p_mod + sid->inf.mpc_pressure_spring_reduction
    );
}

int16_t CrossoverShift::calc_momentum_overlap_2() {
    int ret = this->get_trq_adder_map_val() + this->torque_req_val + this->emergency_trq_val;
    int reduction = ((float)this->emergency_trq_val*100) / pm->release_coefficient();
    ret = MAX(0, ret-reduction);
    if (
        (this->upshifting && sd->input_torque <= 0) ||
        (!this->upshifting && sd->input_torque >= 0)
    ) {
        ret += (abs_input_trq*2);
    }
    ret -= this->trq_req_compensate_val;
    return MAX(0, ret);
}

int16_t CrossoverShift::get_and_set_adapt_rpm_off_clutch() {
    // Make note of the negative maximum
    if (sid->ptr_r_clutch_speeds->off_clutch_speed < rpm_adapt_off_clutch) {
        rpm_adapt_off_clutch = sid->ptr_r_clutch_speeds->off_clutch_speed;
        return 0;
    } else {
        return sid->ptr_r_clutch_speeds->off_clutch_speed - rpm_adapt_off_clutch;
    }
}

void CrossoverShift::fill_adapt() {
    if (this->do_fill_time_adaptation && !this->end_of_fill_time_adapt) {
        // Waiting for clutch movement (Checked and verified)
        int16_t rpm_clamped = this->get_and_set_adapt_rpm_off_clutch();
        bool rpm_in_limits = (sd->input_rpm <= sd->engine_rpm+100 && upshifting) || (sd->engine_rpm <= sd->input_rpm+100 && !upshifting);
        if (rpm_clamped > 50 && rpm_in_limits) {
            this->end_of_fill_time_adapt = true;
            this->offset_adapt_timer_by_clutch_delay();
            // Now compare to when the transition happened
            if (this->fill_time_adapt_timer < this->cycles_high_filling) {
                this->result_fill_time_adaptation = -1;
            } else {
                if (this->fill_time_adapt_timer < this->cycles_high_filling + this->cycles_ramp_to_low_filling) {
                    this->result_fill_time_adaptation = -1;
                } else {
                    this->result_fill_time_adaptation = this->calc_t_adapt_offset_adv(this->fill_time_adapt_timer);
                }
            }
            ESP_LOGI("ADAPT", "FillAdapt end in Filling phase. Res %d", this->result_fill_time_adaptation);
            if (sid->adaptation_mgr) {
                sid->adaptation_mgr->offset_prefill_cycles(sid->inf.map_idx, this->result_fill_time_adaptation);
            }
        }
    }
}

void CrossoverShift::overlap_adapt() {
    if (this->do_fill_time_adaptation && !this->end_of_fill_time_adapt) {
        // Waiting for clutch movement (Checked and verified)
        int16_t rpm_clamped = this->get_and_set_adapt_rpm_off_clutch();
        int max_trq = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, sid->prefill_info.low_fill_pressure_on_clutch, CoefficientTy::Sliding);
        if (abs_input_trq <= max_trq) {
            bool rpm_in_limits = (sd->input_rpm <= sd->engine_rpm && upshifting) || (sd->engine_rpm <= sd->input_rpm && !upshifting);
            if (rpm_clamped > 50 && rpm_in_limits) {
                this->end_of_fill_time_adapt = true;
                this->offset_adapt_timer_by_clutch_delay();
                // Now compare to when the transition happened
                this->result_fill_time_adaptation = this->calc_t_adapt_offset_adv(this->fill_time_adapt_timer);
                ESP_LOGI("ADAPT", "FillAdapt end in overlap phase. Res %d", this->result_fill_time_adaptation);
                if (sid->adaptation_mgr) {
                    sid->adaptation_mgr->offset_prefill_cycles(sid->inf.map_idx, this->result_fill_time_adaptation);
                }
            }
        } else {
            // Cancel adaptation
            this->do_fill_time_adaptation = false;
            ESP_LOGI("ADAPT", "FillAdapt CANCELLED in overlap phase");
        }
    }
}

void CrossoverShift::overlap2_adapt() {
    if (this->do_fill_time_adaptation && !this->end_of_fill_time_adapt) {
        // Now we are FAR too late, so we must increase pressure
        int max_trq = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, sid->prefill_info.low_fill_pressure_on_clutch, CoefficientTy::Sliding);
        if (max_trq < abs_input_trq + VEHICLE_CONFIG.engine_drag_torque/10.0 || sd->output_rpm < 150) {
            // Cancel adaptations
            this->do_fill_time_adaptation = false;
            ESP_LOGI("ADAPT", "FillAdapt CANCELLED in overlap2 phase");
        } else {
            this->end_of_fill_time_adapt = true;
            this->offset_adapt_timer_by_clutch_delay();
            this->result_fill_time_adaptation = this->calc_t_adapt_offset_adv(this->fill_time_adapt_timer);
            ESP_LOGI("ADAPT", "FillAdapt end in overlap2 phase. Res %d", this->result_fill_time_adaptation);
            if (sid->adaptation_mgr) {
                sid->adaptation_mgr->offset_prefill_cycles(sid->inf.map_idx, this->result_fill_time_adaptation);
            }
        }
    }
}

int8_t CrossoverShift::calc_t_adapt_offset_adv(int8_t cycle_change) {
    float sqrt_high_p = sqrt((float)sid->prefill_info.fill_pressure_on_clutch);
    float sqrt_low_p = sqrt((float)sid->prefill_info.low_fill_pressure_on_clutch);
    float cycles_high = (float)cycles_high_filling;

    float delta = sqrt_low_p * 
        (float)((cycle_change - cycles_high) - this->cycles_ramp_to_low_filling - (this->cycles_low_filling/2.0));
    delta /= sqrt_high_p;

    int8_t res = 0;
    if (delta >= 1.0) {
        res = 1;
    } else if (delta <= -1.0) {
        res = -1;
    }
    return res;
}

void CrossoverShift::offset_adapt_timer_by_clutch_delay() {
    const uint16_t SPRING_BITMASK = 0x8A;
    uint8_t delay = 5; // Clutches with spring
    if ((SPRING_BITMASK & (1 << sid->inf.map_idx)) == 0) {
        delay = 4; // Clutches without spring
    }
    if (this->fill_time_adapt_timer > delay) {
        this->fill_time_adapt_timer -= delay;
    } else {
        this->fill_time_adapt_timer = 0;
    }
}

int16_t CrossoverShift::calculate_dynamic_inertia() {
    
    // There is filtering logic in EGS logic, but it is always hard coded to 0 (No filtering)
    if (sid->change == GearChange::_1_2) {
        return 0;
    } else {
        int delta = (int)sd->engine_rpm - (int)this->old_engine_rpm;
        int inertia = (delta)*(VEHICLE_CONFIG.engine_drag_torque/10);
        inertia /= 20;
        this->dyn_inertia_filtered = first_order_filter(4, inertia*10, this->dyn_inertia_filtered);
        return this->dyn_inertia_filtered/10;
    }
}