#include "shift_crossover.h"

const uint8_t PHASE_BLEED     = 0;
const uint8_t PHASE_PREFILL_1 = 1;
const uint8_t PHASE_PREFILL_2 = 2;
const uint8_t PHASE_PREFILL_3 = 3;

const uint8_t PHASE_OVERLAP = 4;

const uint8_t PHASE_MOMENTUM_RAMP = 5;
const uint8_t PHASE_MOMENTUM_HOLD = 6;
const uint8_t PHASE_MAX_PRESSURE  = 7;
const uint8_t PHASE_END_CONTROL   = 8;

const uint8_t FILL_RAMP_TIME = 60;
const uint8_t FILL_HOLD_TIME = 100;

CrossoverShift::CrossoverShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {}

CrossoverShift::~CrossoverShift() {

}

uint8_t CrossoverShift::step(
    uint8_t phase_id,
    uint16_t abs_input_torque,
    bool stationary,
    bool is_upshift,
    uint16_t phase_elapsed,
    uint16_t total_elapsed,
    PressureManager* pm,
    SensorData* sd
) {
    uint8_t ret = STEP_RES_CONTINUE;
    int centrifugal_force_on_clutch = pm->calculate_centrifugal_force_for_clutch(sid->applying, sd->input_rpm, MAX(0, sid->ptr_r_clutch_speeds->rear_sun_speed));
    int centrifugal_force_off_clutch = pm->calculate_centrifugal_force_for_clutch(sid->releasing, sd->input_rpm, MAX(0, sid->ptr_r_clutch_speeds->rear_sun_speed));
    ShiftPressures* p_now = sid->ptr_w_pressures;
    if (phase_id == PHASE_BLEED) {
        int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, MAX(abs_input_torque, 30));
        // Clutches
        p_now->on_clutch = sid->prefill_info.fill_pressure_on_clutch;
        p_now->off_clutch = wp_old_clutch;
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = interpolate_float(phase_elapsed, sid->SPC_MAX, p_now->overlap_shift - centrifugal_force_on_clutch, 0, 100, InterpType::Linear);
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
        if (phase_elapsed >= 100) {
            // Turn on the switching valve!
            pressure_manager->set_shift_circuit(sid->inf.shift_circuit, true);
            sid->tcc->set_shift_target_state(InternalTccState::Open);
            ret = PHASE_PREFILL_1;
        }
    } else if (phase_id == PHASE_PREFILL_1) {
        int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, MAX(abs_input_torque, 30));
        // Clutches
        p_now->on_clutch = sid->prefill_info.fill_pressure_on_clutch;
        p_now->off_clutch = wp_old_clutch;
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
        // Return condition
        if (phase_elapsed > sid->prefill_info.fill_time) {
            if (abs_input_torque > VEHICLE_CONFIG.engine_drag_torque/10.0) {
                this->do_high_filling = true;
            }
            ret = PHASE_PREFILL_2;
        }
        if (sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
            // Jump to the overlap phase as filling has finished early!
            ret = PHASE_OVERLAP;
        }
    } else if (phase_id == PHASE_PREFILL_2) {
        int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, MAX(abs_input_torque, 30));
        int targ_p = 0;
        if (this->do_high_filling) {
            targ_p = sid->prefill_info.low_fill_pressure_on_clutch;
        }
        // Clutches
        p_now->on_clutch = interpolate_float(phase_elapsed, sid->prefill_info.fill_pressure_on_clutch, targ_p, 0, FILL_RAMP_TIME, InterpType::Linear);
        p_now->off_clutch = wp_old_clutch;
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
        // Return condition
        if (sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
            // Jump to the overlap phase as filling has finished early!
            ret = PHASE_OVERLAP;
        }
        if (phase_elapsed >= FILL_RAMP_TIME) {
            ret = PHASE_PREFILL_3;
        }
    } else if (phase_id == PHASE_PREFILL_3) {
        int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, MAX(abs_input_torque, 30));
        int targ_p = 0;
        if (this->do_high_filling) {
            targ_p = sid->prefill_info.low_fill_pressure_on_clutch;
        }
        // Clutches
        p_now->on_clutch = targ_p;
        p_now->off_clutch = wp_old_clutch;
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
        // Return condition
        if (sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
            // Jump to the overlap phase as filling has finished early!
            ret = PHASE_OVERLAP;
        }
        if (phase_elapsed >= FILL_HOLD_TIME) {
            ret = PHASE_OVERLAP;
        }
    } else if (phase_id == PHASE_OVERLAP) {
        int wp_new_clutch = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque, false);
        // Shift circuit
        p_now->on_clutch = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->on_clutch, wp_new_clutch, 0, 500, InterpType::Linear);
        // for modulating solenoid adjustment
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
        // mod circuit
        if (this->do_high_filling) {
            // Higher pressure filling requires turning off the clutch slowly as applying clutch is already pressurized
            int spc_at_end = MAX(wp_new_clutch, sid->ptr_prev_pressures->on_clutch);
            int torque_held_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch);
            int torque_left = MAX(0, MAX(30, abs_input_torque) - torque_held_new_clutch);
            p_now->off_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, torque_left);
            p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;

            int mod_sol_holding = ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
                sid->inf.mpc_pressure_spring_reduction;
            
            int mod_sol_min = ((spc_at_end - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((sid->spring_off_clutch - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
                sid->inf.mpc_pressure_spring_reduction;
            p_now->mod_sol_req = MAX(mod_sol_holding, mod_sol_min);
        } else {
            // Low pressure filling requires less pressure as the applying clutch starts at 0mBar
            p_now->off_clutch = 0;
            p_now->overlap_mod = 0;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((-centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
                sid->inf.mpc_pressure_spring_reduction
            , 0);
        }
        // Next phase once clutch is moving, or 500ms
        if (phase_elapsed >= 500 || sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
            ret = PHASE_MOMENTUM_RAMP;
        }
    } else if (phase_id == PHASE_MOMENTUM_RAMP) {
        int start_p = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque, false);
        uint16_t e_trq = pressure_manager->find_decent_adder_torque(sid->change, abs(sd->static_torque), sd->output_rpm);
        int wp_new_clutch = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque+e_trq, false);
        // Clutches
        sid->ptr_w_pressures->on_clutch = interpolate_float(phase_elapsed, start_p, wp_new_clutch, 0, sid->chars.target_shift_time/4, InterpType::Linear);
        sid->ptr_w_pressures->off_clutch = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->off_clutch, 0, 0, sid->chars.target_shift_time/2, InterpType::Linear);
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
        int mod_end  = (
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
            ((-centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
            (-(sid->inf.pressure_multi_mpc*150)) + // 150 comes from calibration pointer on E55 coding (TODO - Locate in calibration and assign in CAL structure)
            sid->inf.mpc_pressure_spring_reduction
        );
        p_now->mod_sol_req = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->mod_sol_req, mod_end, 0, sid->chars.target_shift_time/2, InterpType::Linear);
        if (phase_elapsed >= sid->chars.target_shift_time/2 || sid->ptr_r_clutch_speeds->off_clutch_speed > sid->ptr_r_clutch_speeds->on_clutch_speed) {
            ret = PHASE_MOMENTUM_HOLD;
        }
    } else if (phase_id == PHASE_MOMENTUM_HOLD) {
        // TODO (here we should control the decent of the clutch)
        // For now, just ramp up pressures if target has not been achieved
        float inertia = ShiftHelpers::calcualte_abs_engine_inertia(sid->inf.map_idx, sd->engine_rpm, sd->input_rpm);
        int targ_torque = abs_input_torque + inertia;
        if (sd->pedal_pos < 10) {
            if (is_upshift) {
                targ_torque = MIN(inertia, abs_input_torque);
            } else {
                targ_torque += abs_input_torque;
            }
        }
        float adder = pressure_manager->find_decent_adder_torque(sid->change, abs(sd->static_torque), sd->output_rpm);
        if (phase_elapsed > sid->chars.target_shift_time/2) {
            adder += interpolate_float(phase_elapsed-sid->chars.target_shift_time/2, 0, inertia, 0, sid->chars.target_shift_time/2, InterpType::Linear);
        }
        int wp_new_clutch = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, targ_torque + adder, false);  
        // Clutches
        sid->ptr_w_pressures->on_clutch = wp_new_clutch;
        sid->ptr_w_pressures->off_clutch = 0;
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
        p_now->mod_sol_req  = (
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
            ((-centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
            (-(sid->inf.pressure_multi_mpc*150)) + // 150 comes from calibration pointer on E55 coding (TODO - Locate in calibration and assign in CAL structure)
            sid->inf.mpc_pressure_spring_reduction
        );
        // Exit conditions
        if (stationary && phase_elapsed > 1000) { // 1. Stationary shift (can't measure clutch speed)
            ret = PHASE_MAX_PRESSURE;
        } else if (sid->ptr_r_clutch_speeds->on_clutch_speed < 10) {
            ret = PHASE_MAX_PRESSURE;
        }
        // SHIFT FAILURE (NO MOVEMENT!)
        if (!stationary && phase_elapsed > 3000 && sid->ptr_r_clutch_speeds->off_clutch_speed < 100) {
            ret = STEP_RES_FAILURE;
        }
    } else if (phase_id == PHASE_MAX_PRESSURE) {
        // Max pressure phase. Pressures on the applied clutch are ramped up to ensure locking in 2 ramps.
        int wp_new_clutch = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque, false);
        ShiftPressures* p_prev = sid->ptr_prev_pressures;
        // Clutches
        p_now->on_clutch = interpolate_float(phase_elapsed, p_prev->on_clutch, wp_new_clutch, 0, sid->maxp_info.ramp_time, InterpType::Linear);
        p_now->off_clutch = 0;
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = interpolate_float(phase_elapsed, p_prev->overlap_shift, sid->MOD_MAX, 0, sid->maxp_info.ramp_time, InterpType::Linear);
        // DIFFERS! - For mod_sol_req, use a different value for overlap_shift
        int overlap_shift_mod_sol = MIN(p_now->overlap_shift, wp_new_clutch + sid->spring_on_clutch);
        // Solenoids (important!)
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req  = (
            ((overlap_shift_mod_sol - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
            (-centrifugal_force_off_clutch * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
            (-sid->inf.pressure_multi_mpc*150) + // 150 comes from calibration pointer on E55 coding (TODO - Locate in calibration and assign in CAL structure)
            sid->inf.mpc_pressure_spring_reduction
        );
        if (phase_elapsed > sid->maxp_info.hold_time + sid->maxp_info.ramp_time) {
            // Turn off the shift circuit!
            pm->set_shift_circuit(sid->inf.shift_circuit, false);
            sid->tcc->on_shift_ending();
            ret = PHASE_END_CONTROL;
        }
    } else if (phase_id == PHASE_END_CONTROL) {
        // Shift solenoid is off
        int wp_gear = pm->find_working_mpc_pressure(sid->targ_g);
        p_now->on_clutch = 0;
        p_now->off_clutch = 0;
        p_now->overlap_mod = 0;
        p_now->overlap_shift = 0;
        p_now->shift_sol_req = sid->SPC_MAX;
        p_now->overlap_mod = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->mod_sol_req, wp_gear, 0, 250, InterpType::Linear);
        if (phase_elapsed > 250) {
            ret = STEP_RES_END_SHIFT;
        }
    } else {
        ret = STEP_RES_END_SHIFT; // WTF? Should never happen
    }

    // Torque request data
    //if (is_upshift) {
    //    if (phase_id >= PHASE_OVERLAP) {
    //        if (this->trq_req_amount_at_end != 0) {
    //            if (this->torque_req_start_time == 0) {
    //                this->torque_req_start_time = total_elapsed;
    //            }
    //        }
    //        
    //    }
    //}
    return ret;
}

uint8_t CrossoverShift::max_shift_stage_id() {
    return PHASE_MAX_PRESSURE;
}