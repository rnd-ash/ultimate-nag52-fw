#include "shift_release.h"

const uint8_t PHASE_BLEED     = 0;
// Fill and release (FAR) (Same time!)
const uint8_t PHASE_FILL_AND_RELEASE = 1;
// Securing phases
const uint8_t PHASE_OVERLAP = 2;
const uint8_t PHASE_MAX_PRESSURE  = 3;
const uint8_t PHASE_END_CONTROL   = 4;

const uint8_t FILL_RAMP_TIME = 60;
const uint8_t FILL_HOLD_TIME = 100;

ReleasingShift::ReleasingShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {}
ReleasingShift::~ReleasingShift() {}

uint8_t ReleasingShift::step(
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
    // These 2 vars are used for torque requests
    //int max_trq_on_clutch  = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, sid->ptr_w_pressures->on_clutch);
    int max_trq_off_clutch = pm->calc_max_torque_for_clutch(sid->curr_g, sid->releasing, sid->ptr_w_pressures->off_clutch, true);

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
            InternalTccState now = sid->tcc->__get_internal_state();
            sid->tcc->set_shift_target_state(sd, now); // Prevent TCC from locking more than current state
            this->subphase_mod = 0;
            this->subphase_shift = 0;
            ret = PHASE_FILL_AND_RELEASE;
        }
    } else if (phase_id == PHASE_FILL_AND_RELEASE) {
        // Shift pressure
        if (0 == this->subphase_shift) { // High pressure fill
            uint16_t elapsed = phase_elapsed - this->ts_phase_shift;
            p_now->on_clutch = sid->prefill_info.fill_pressure_on_clutch;
            p_now->overlap_shift = sid->spring_on_clutch + p_now->on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (elapsed > sid->prefill_info.fill_time) {
                this->subphase_shift = 1;
                this->ts_phase_shift = phase_elapsed;
            }
            this->torque_at_new_clutch = 0;
        } else if (1 == this->subphase_shift) { // Ramp to low pressure
#define RAMP_TIME 60
            uint16_t elapsed = phase_elapsed - this->ts_phase_shift;
            p_now->on_clutch = interpolate_float(elapsed, sid->prefill_info.fill_pressure_on_clutch, sid->prefill_info.low_fill_pressure_on_clutch, 0, RAMP_TIME, InterpType::Linear);
            p_now->overlap_shift = sid->spring_on_clutch + p_now->on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (elapsed > RAMP_TIME) {
                this->subphase_shift = 2;   
                this->ts_phase_shift = phase_elapsed;
            }
            this->torque_at_new_clutch = 0;
        } else if (2 == this->subphase_shift) { // Low pressure filling
#define HOLD_3_TIME 100
            uint16_t elapsed = phase_elapsed - this->ts_phase_shift;
            p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch;
            p_now->overlap_shift = sid->spring_on_clutch + p_now->on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (elapsed > HOLD_3_TIME) {
                this->subphase_shift = 3;
                this->ts_phase_shift = phase_elapsed;
            }
            this->torque_at_new_clutch = 0;
        } else if (3 == this->subphase_shift) { // Waiting for a clutch to move
            uint16_t elapsed = phase_elapsed - this->ts_phase_shift;
            p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch;
            p_now->overlap_shift = sid->spring_on_clutch + p_now->on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (abs(sid->ptr_r_clutch_speeds->off_clutch_speed) > 100 || elapsed > 1000) {
                this->subphase_shift = 4;
                this->ts_phase_shift = phase_elapsed;
                this->rpm_shift_phase_3 = sid->ptr_r_clutch_speeds->off_clutch_speed;
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, true);
        } else if (4 == this->subphase_shift) { // Ramping new clutch
            uint16_t elapsed = phase_elapsed - this->ts_phase_shift;
            this->filling_adder += 8;
            p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch + filling_adder;
            p_now->overlap_shift = sid->spring_on_clutch + p_now->on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            int end = (float)sid->ptr_r_pre_clutch_speeds->on_clutch_speed * 0.75;
            if (sid->ptr_r_clutch_speeds->off_clutch_speed > this->rpm_shift_phase_3 + 100 || elapsed > 1000) {
                this->subphase_shift = 5;
                this->ts_phase_shift = phase_elapsed;
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, true);
        } else if (5 == this->subphase_shift) { // Holding upper pressure
            uint16_t elapsed = phase_elapsed - this->ts_phase_shift;
            p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch + filling_adder;
            p_now->overlap_shift = sid->spring_on_clutch + p_now->on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (sid->ptr_r_clutch_speeds->on_clutch_speed < 100 || elapsed > 1000) {
                // END FILLING AND RELEASE
                ret = PHASE_OVERLAP;
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, true);
        } else {
            ret = PHASE_OVERLAP; // WTF
        }
        // Mod pressure
        if (0 == this->subphase_mod) {
            uint16_t elapsed = phase_elapsed - this->ts_phase_mod;
            if (-1 == mod_time_phase_0) { // Not initialized
                int max_time = sid->prefill_info.fill_time + RAMP_TIME + HOLD_3_TIME;
                if (sid->change == ProfileGearChange::TWO_ONE || sid->change == ProfileGearChange::THREE_TWO) {
                    max_time = 0;
                } else {
                    float inertia = ShiftHelpers::calcualte_abs_engine_inertia(sid->inf.map_idx, sd->engine_rpm, sd->input_rpm);
                    float free_torque = pm->find_freeing_torque(sid->change, sd->static_torque, sd->output_rpm);
                    float drag = pm->find_turbine_drag(sid->inf.map_idx);
                    
                    int reduction = inertia * (float)(sid->ptr_r_clutch_speeds->on_clutch_speed) / free_torque / drag;
                    max_time = MAX(max_time, max_time - reduction);
                }
                this->mod_time_phase_0 = max_time;
            }

            int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, MAX(abs_input_torque, 30));

            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            if (elapsed > this->mod_time_phase_0 || sid->ptr_r_clutch_speeds->on_clutch_speed < -100) {
                this->subphase_mod = 1;
                this->ts_phase_mod = phase_elapsed;
            }
        } else if (1 == this->subphase_mod) {
            uint16_t elapsed = phase_elapsed - this->ts_phase_mod;
#define MOD_RAMP_TIME 80
            float free_torque = pm->find_freeing_torque(sid->change, sd->static_torque, sd->output_rpm);
            int wp_old_clutch = pm->find_working_pressure_for_clutch(sid->curr_g, sid->releasing, MAX(0, abs_input_torque - free_torque), false);
            p_now->off_clutch = interpolate_float(elapsed, sid->ptr_prev_pressures->off_clutch, wp_old_clutch, 0, MOD_RAMP_TIME, InterpType::Linear);
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            if (
                elapsed > MOD_RAMP_TIME ||
                sid->ptr_r_clutch_speeds->on_clutch_speed < 100 ||
                sid->ptr_r_clutch_speeds->off_clutch_speed > 150
            ) {
                this->subphase_mod = 2;
                this->ts_phase_mod = phase_elapsed;
            }
        } else if (2 == this->subphase_mod) {
            uint16_t elapsed = phase_elapsed - this->ts_phase_mod;
            float free_torque = pm->find_freeing_torque(sid->change, sd->static_torque, sd->output_rpm);
            float sports_adder = interpolate_float(sd->pedal_pos, 1.0, 2.0, 50, 200, InterpType::Linear);
            this->filling_trq_reducer += 9.0 * sports_adder;
            int trq = MAX(0, abs_input_torque - free_torque - filling_trq_reducer);

            int wp_old_clutch = pm->find_working_pressure_for_clutch(sid->curr_g, sid->releasing, trq, false);

            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            if (
                elapsed > 500 ||
                sid->ptr_r_clutch_speeds->off_clutch_speed > 100
            ) {
                this->subphase_mod = 3;
                this->ts_phase_mod = phase_elapsed;
            }
            
        } else if (3 == this->subphase_mod) {
            // Hold pressure
            float free_torque = pm->find_freeing_torque(sid->change, sd->static_torque, sd->output_rpm);
            int trq = MAX(0, abs_input_torque - free_torque - filling_trq_reducer);
            int wp_old_clutch = pm->find_working_pressure_for_clutch(sid->curr_g, sid->releasing, trq, false);
            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
                sid->inf.mpc_pressure_spring_reduction
            , 0);
            // Crossover point, start sync.
            if (sid->ptr_r_clutch_speeds->on_clutch_speed < sid->ptr_r_pre_clutch_speeds->on_clutch_speed/2) {
                this->subphase_mod = 4;
                this->ts_phase_mod = phase_elapsed;
            }
        } else if (4 == this->subphase_mod) {
#define MOD_RAMP_4_TIME 80
            uint16_t elapsed = phase_elapsed - this->ts_phase_mod;
            // No exit (Governed by shift pressure)
            float fr_fo = pm->release_coefficient() / pm->friction_coefficient();
            float free_torque = pm->find_freeing_torque(sid->change, sd->static_torque, sd->output_rpm);
            int trq_off_clutch = MAX(0, abs_input_torque - (((1.0-sid->inf.centrifugal_factor_off_clutch)*free_torque) + (fr_fo*this->torque_at_new_clutch)));
            int wp_old_clutch = pm->find_working_pressure_for_clutch(sid->curr_g, sid->releasing, trq_off_clutch, false);

            p_now->off_clutch = interpolate_float(elapsed, sid->ptr_prev_pressures->on_clutch, wp_old_clutch, 0, MOD_RAMP_4_TIME, InterpType::Linear);
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
                sid->inf.mpc_pressure_spring_reduction
            , 0);
        }
    } else if (phase_id == PHASE_OVERLAP) {
        int duration = MIN(100, sid->chars.target_shift_time/2);
        // New clutch gets full pressure (+momentum)
        // Old clutch valve is empties completely
        int wp_new_clutch = pm->find_releasing_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque) + filling_adder;
        p_now->on_clutch = MAX(sid->ptr_prev_pressures->on_clutch, interpolate_float(phase_elapsed, sid->ptr_prev_pressures->on_clutch, wp_new_clutch, 0, duration, InterpType::Linear));
        p_now->off_clutch = 0;
        p_now->overlap_mod = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->off_clutch, 0, 0, duration, InterpType::Linear);
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;

        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);

        if (phase_elapsed > duration) {
            torque_adder += 2;
        }

        if (sid->ptr_r_clutch_speeds->on_clutch_speed < 100) {
            ret = PHASE_MAX_PRESSURE;
        }
    } else if (phase_id == PHASE_MAX_PRESSURE) {
        // Max pressure phase. Pressures on the applied clutch are ramped up to ensure locking in 2 ramps.
        int wp_new_clutch = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque, false);
        ShiftPressures* p_prev = sid->ptr_prev_pressures;
        // Clutches
        p_now->on_clutch = interpolate_float(phase_elapsed, p_prev->on_clutch, MAX(wp_new_clutch, p_prev->on_clutch), 0, sid->maxp_info.ramp_time, InterpType::Linear);
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
        p_now->mod_sol_req = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->mod_sol_req, wp_gear, 0, 250, InterpType::Linear);
        if (phase_elapsed > 250) {
            ret = STEP_RES_END_SHIFT;
        }
    } else {
        ret = STEP_RES_END_SHIFT; // WTF? Should never happen
    }

     // Limiting engine torque by this much (Computed later with indicated_torque - trq_req_targ = trq request output)
    int trq_req_targ = sd->static_torque;
    // We can only request if engine torque is above engines min torque
    if (sd->static_torque_wo_request > sd->min_torque && sd->driver_requested_torque > sd->min_torque && sd->input_rpm > 800 && this->trq_req) {
        bool inc = phase_id == PHASE_MAX_PRESSURE;

        int target_for_freeing = sd->driver_requested_torque - pm->find_freeing_torque(sid->change, sd->static_torque_wo_request, sd->output_rpm);

        // Calc clutch maximums
        int max_for_clutch = pm->calc_max_torque_for_clutch(sid->curr_g, sid->releasing, p_now->off_clutch, true);
        if (this->torque_at_new_clutch > max_for_clutch) { // So the on clutch takes over when off clutch is released below handling point
            max_for_clutch = this->torque_at_new_clutch;
        }
        int target = MIN(target_for_freeing, max_for_clutch);
        if (phase_id == PHASE_FILL_AND_RELEASE && this->subphase_mod >= 1) {
            // Ramp down + Hold
            if (trq_req_start_time == 0) {
                trq_req_start_time = total_elapsed;
            }
            int into_down_ramp = total_elapsed - trq_req_start_time;
            trq_req_targ = interpolate_float(into_down_ramp, sd->driver_requested_torque, target, 0, MOD_RAMP_TIME*2, InterpType::Linear);
            this->last_trq_req = trq_req_targ;
            this->torque_req_en = true;
        } else if (this->torque_req_en) {
            // Ramp up
            if (trq_req_end_time == 0) {
                trq_req_end_time = total_elapsed;
            }
            int into_up_ramp = total_elapsed - trq_req_start_time;
            int start_rpm = sid->ptr_r_pre_clutch_speeds->on_clutch_speed/2;
            trq_req_targ = interpolate_float(sid->ptr_r_clutch_speeds->on_clutch_speed, this->last_trq_req, sd->driver_requested_torque, start_rpm, 0, InterpType::Linear);
        }
        if (!torque_req_en || (trq_req_targ >= sd->static_torque_wo_request || trq_req_targ >= sd->driver_requested_torque)) {
            // No request required
            sid->ptr_w_trq_req->amount = 0;
            sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
            sid->ptr_w_trq_req->ty = TorqueRequestControlType::None;
        } else {
            // Request required
            sid->ptr_w_trq_req->amount = trq_req_targ;
            sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
            sid->ptr_w_trq_req->ty = inc ? TorqueRequestControlType::BackToDemandTorque : TorqueRequestControlType::NormalSpeed;
        }
    } else {
        // DISABLE REQUESTS (REQ OUT OF BOUNDS FOR ENGINE)!
        sid->ptr_w_trq_req->amount = 0;
        sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
        sid->ptr_w_trq_req->ty = TorqueRequestControlType::None;
    }

    return ret;
}

uint8_t ReleasingShift::max_shift_stage_id() {
    return PHASE_MAX_PRESSURE;
}