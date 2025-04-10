#include "shift_release.h"

const uint8_t PHASE_BLEED     = 0;
const uint8_t PHASE_FILL_AND_RELEASE = 1;
const uint8_t PHASE_MAX_PRESSURE  = 2;
const uint8_t PHASE_END_CONTROL   = 3;

const uint8_t FILL_RAMP_TIME = 60;
const uint8_t FILL_HOLD_TIME = 100;

ReleasingShift::ReleasingShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {
    this->on_clutch_delta = new FirstOrderAverage(5);
}
ReleasingShift::~ReleasingShift() {
    delete this->on_clutch_delta;
}

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
    float drag = pm->find_turbine_drag(sid->inf.map_idx);
    this->inertia = ShiftHelpers::calcualte_abs_engine_inertia(sid->inf.map_idx, sd->engine_rpm, sd->input_rpm);
    // Keep calculating these values until we have to start using them
    if (phase_id == PHASE_BLEED) {
        float max = interpolate_float(sid->targ_time, 2.0, 3.0, 1000, 100, InterpType::Linear);
        this->sports_factor = interpolate_float(sd->pedal_delta->get_average(), 1.0, max, 10, 200, InterpType::Linear); //10%/sec - 200%/sec
    }
    // Threshold RPM for ramping up based on torque
    //if (phase_id == 0) {
        int freeing_torque = pm->find_freeing_torque(sid->change, sd->converted_torque, sd->output_rpm);
        // Calculate torque request trq here. (Only used until we ramp down torque
        int trq_request_raw = freeing_torque;
        if (is_upshift) {
            trq_request_raw = trq_request_raw;
        } else {
            // Downshift uses pedal multiplier
            trq_request_raw = this->sports_factor * trq_request_raw;
        }
        trq_request_raw = MIN(trq_request_raw + this->trq_req_adder, abs_input_torque);
        freeing_torque = MIN(this->sports_factor * freeing_torque, abs_input_torque);
        this->freeing_torque_calc = freeing_torque;
        int effective_torque = MIN(freeing_torque,
                                    (freeing_torque + this->torque_at_new_clutch) / 2);
        //this->threshold_rpm =
        //        (effective_torque + this->torque_at_new_clutch) *
        //        ((80.0 + 40.0) / 1000.0) *
        //        // 80 for MPC ramp time, 20*2 (40) for computation delay over CAN (Rx of Sta. Trq -> Tx of EGS Trq)
        //        drag /
        //        inertia;
        //this->threshold_rpm = MAX(this->threshold_rpm, 100);
        this->threshold_rpm = ShiftHelpers::get_threshold_rpm(sid->inf.map_idx, effective_torque + this->torque_at_new_clutch, 4); // 4=80ms (20ms*4)
    //}

    ShiftPressures* p_now = sid->ptr_w_pressures;
    if (phase_id == PHASE_BLEED) {
        int wp_old_clutch = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, MAX(abs_input_torque, 30), CoefficientTy::Release);
        // EGS52 does this, calculates the ramp, and then offsets it by centrifugal force
        p_now->on_clutch = sid->prefill_info.fill_pressure_on_clutch + sid->release_spring_on_clutch;
        p_now->shift_sol_req = MAX(0, interpolate_float(phase_elapsed, sid->SPC_MAX, p_now->on_clutch, 0, 100, InterpType::Linear) - centrifugal_force_on_clutch);
        // Mod circuit (Hold)
        p_now->off_clutch = wp_old_clutch;
        p_now->overlap_mod = p_now->off_clutch + sid->release_spring_off_clutch;
        p_now->mod_sol_req = MAX(
            ((MAX(0, p_now->on_clutch - centrifugal_force_on_clutch)) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
        this->torque_at_old_clutch = abs_input_torque;
        if (phase_elapsed >= 100) {
            // Turn on the switching valve!
            pressure_manager->set_shift_circuit(sid->inf.shift_circuit, true);
            this->subphase_mod = 0;
            this->subphase_shift = 0;
            ret = PHASE_FILL_AND_RELEASE;
        }
    } else if (phase_id == PHASE_FILL_AND_RELEASE) {
        int elapsed_shift = this->elapsed_subphase_shift(phase_elapsed);
        int elapsed_mod = this->elapsed_subphase_mod(phase_elapsed);
        // Shift pressure
        if (0 == this->subphase_shift) { // High pressure fill
            p_now->on_clutch = sid->prefill_info.fill_pressure_on_clutch + sid->release_spring_on_clutch;
            p_now->shift_sol_req = MAX(0, p_now->on_clutch - centrifugal_force_on_clutch);
            if (elapsed_shift > sid->prefill_info.fill_time) {
                this->inc_subphase_shift(phase_elapsed);
                this->low_fill_p = MAX(
                    sid->prefill_info.low_fill_pressure_on_clutch,
                    (
                        pressure_manager->p_clutch_with_coef(sid->targ_g, sid->applying, abs_input_torque, CoefficientTy::Release) +
                        sid->prefill_info.low_fill_pressure_on_clutch
                    )/2
                );
            }
            this->torque_at_new_clutch = 0;
        } else if (1 == this->subphase_shift) { // Ramp to low pressure
#define RAMP_TIME 80
            int targ = this->low_fill_p + sid->release_spring_on_clutch;
            p_now->on_clutch = interpolate_float(elapsed_shift, sid->prefill_info.fill_pressure_on_clutch, targ, 0, RAMP_TIME, InterpType::Linear);
            p_now->shift_sol_req = MAX(0, p_now->on_clutch - centrifugal_force_on_clutch);
            if (elapsed_shift >= RAMP_TIME) {
                this->inc_subphase_shift(phase_elapsed);
            }
            this->torque_at_new_clutch = 0;
        } else if (2 == this->subphase_shift) { // Low pressure filling
#define HOLD_3_TIME 40
            p_now->on_clutch = this->low_fill_p + sid->release_spring_on_clutch;
            p_now->shift_sol_req = MAX(0, p_now->on_clutch - centrifugal_force_on_clutch);
            if (elapsed_shift >= HOLD_3_TIME) {
                this->inc_subphase_shift(phase_elapsed);
            }
            this->torque_at_new_clutch = 0;
        } else if (3 == this->subphase_shift) { // Waiting for a clutch to move
            p_now->on_clutch = this->low_fill_p + sid->release_spring_on_clutch;
            p_now->shift_sol_req = MAX(0, p_now->on_clutch - centrifugal_force_on_clutch);
            bool sync_with_mod = phase_elapsed >= (this->mod_time_phase_0 + this->mod_time_phase_1);
            // Sync with mod reached, but no speed difference
            // Or timeout
            // Or new clutch speed is near target
            if (
                (sid->ptr_r_clutch_speeds->off_clutch_speed < 100 && sync_with_mod) || // No movement when mod is finished (Need to ramp pressure)
                (sid->ptr_r_clutch_speeds->on_clutch_speed < 100) || // Clutch jumped!
                elapsed_shift > 20 // Timeout on phase
            ) {
                this->inc_subphase_shift(phase_elapsed);
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, CoefficientTy::Sliding);
        } else if (4 == this->subphase_shift) { // Ramping new clutch (Clutch is still not moving)
            if (sid->profile == race) {
                this->filling_adder += MAX(
                    8.0,
                    pm->p_clutch_with_coef(sid->targ_g, sid->applying, 8, CoefficientTy::Sliding)
                );
            } else {
                this->filling_adder += 8.0;
            }
            p_now->on_clutch = this->low_fill_p + filling_adder + sid->release_spring_on_clutch;
            p_now->shift_sol_req = MAX(0, p_now->on_clutch - centrifugal_force_on_clutch);
            if (
                sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm|| // Trigger syncronize phase
                elapsed_shift > 2500 // Timeout
            ) {
                this->inc_subphase_shift(phase_elapsed);
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, CoefficientTy::Sliding);
            this->trq_req_adder += 0.5;
        } else if (5 == this->subphase_shift) { // Holding upper pressure
            p_now->on_clutch = this->low_fill_p + filling_adder + sid->release_spring_on_clutch;
            p_now->shift_sol_req = MAX(0, p_now->on_clutch - centrifugal_force_on_clutch);
            if (sid->ptr_r_clutch_speeds->on_clutch_speed < 100 || elapsed_shift >= 2500) {
                ret = PHASE_MAX_PRESSURE;
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, CoefficientTy::Sliding);
        } else {
            ret = PHASE_MAX_PRESSURE; // WTF
        }
        // Mod pressure
        if (0 == this->subphase_mod) {
            int max_time = sid->prefill_info.fill_time + RAMP_TIME + HOLD_3_TIME;
            if (sid->change == GearChange::_2_1 || sid->change == GearChange::_3_2) {
                max_time = 0;
            } else {
                int reduction = 0;
                if (0 != freeing_torque_calc) {
                    reduction = inertia * (float)(sid->ptr_r_clutch_speeds->on_clutch_speed) / drag / (float)freeing_torque_calc;
                }
                max_time = MAX(max_time, max_time - reduction);
            }
            this->mod_time_phase_0 = max_time;

            int wp_old_clutch = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, MAX(abs_input_torque, 30), CoefficientTy::Release);

            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->release_spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                (p_now->shift_sol_req * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            this->torque_at_old_clutch = MAX(abs_input_torque, 30);
            if (elapsed_mod > this->mod_time_phase_0 || sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
                this->inc_subphase_mod(phase_elapsed);
            }
        } else if (1 == this->subphase_mod) {
            if (this->mod_time_phase_1 == -1) {
                #define MOD_RAMP_TIME 80
                // Initialize ramp time
                this->mod_time_phase_1 = MAX(   
                    MOD_RAMP_TIME,
                    sid->prefill_info.fill_time + RAMP_TIME - this->mod_time_phase_0
                );
                if (sid->change == GearChange::_2_1 || sid->change == GearChange::_3_2) {
                    this->mod_time_phase_1 = MOD_RAMP_TIME;
                }
            }

            int wp_old_clutch = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, MAX(0, abs_input_torque - this->freeing_torque_calc), CoefficientTy::Sliding);
            p_now->off_clutch = interpolate_float(elapsed_mod, sid->ptr_prev_pressures->off_clutch, wp_old_clutch, 0, this->mod_time_phase_1, InterpType::Linear);
            p_now->overlap_mod = sid->release_spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                (p_now->shift_sol_req * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            if (sid->change == GearChange::_3_2) {
                p_now->mod_sol_req = MAX(
                    (p_now->shift_sol_req/2 * sid->inf.pressure_multi_spc)+
                    ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                    sid->inf.mpc_pressure_spring_reduction
                    , 0);
            }
            if (
                elapsed_mod > this->mod_time_phase_1 ||
                    (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm &&
                    sid->ptr_r_clutch_speeds->off_clutch_speed > 100)
                || sid->ptr_r_clutch_speeds->on_clutch_speed < 100
            ) {
                this->inc_subphase_mod(phase_elapsed);
            }
            this->torque_at_old_clutch = pm->calc_max_torque_for_clutch(sid->curr_g, sid->releasing,  p_now->off_clutch, CoefficientTy::Sliding);
        } else if (2 == this->subphase_mod) {
            // Reducing until releasing the clutch
            this->filling_trq_reducer += (10.0 * this->sports_factor) + (this->sports_factor * 0.05 * this->filling_trq_reducer);
            int trq = MAX(0, abs_input_torque - this->freeing_torque_calc - filling_trq_reducer);
            int wp_old_clutch = 0;
            wp_old_clutch = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, trq, CoefficientTy::Sliding);
            
            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->release_spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                (p_now->shift_sol_req * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            if (sid->change == GearChange::_3_2) {
                p_now->mod_sol_req = MAX(
                    (p_now->shift_sol_req/2 * sid->inf.pressure_multi_spc)+
                    ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                    sid->inf.mpc_pressure_spring_reduction
                    , 0);
            }
            if (
                trq <= 0 || // No more torque to reduce by
                (sid->ptr_r_clutch_speeds->off_clutch_speed > 100) || // Released old clutch
                (sid->ptr_r_clutch_speeds->on_clutch_speed < 100) // Early sync.
            ) {
                this->inc_subphase_mod(phase_elapsed);
            }
            this->torque_at_old_clutch = pm->calc_max_torque_for_clutch(sid->curr_g, sid->releasing,  p_now->off_clutch, CoefficientTy::Sliding);
        } else if (3 == this->subphase_mod) {
            // Hold pressure until we can sync. 
            int trq = MAX(0, abs_input_torque - this->freeing_torque_calc - filling_trq_reducer);
            int wp_old_clutch = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, trq, CoefficientTy::Sliding);
            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->release_spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                (p_now->shift_sol_req * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
            , 0);
            if (sid->change == GearChange::_3_2) {
                p_now->mod_sol_req = MAX(
                    (p_now->shift_sol_req/2 * sid->inf.pressure_multi_spc)+
                    ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                    sid->inf.mpc_pressure_spring_reduction
                , 0);
            }
            // Crossover point, start sync.
            if (
                (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm) // Begin merging of clutches
            ) {
                this->inc_subphase_mod(phase_elapsed);
            }
            this->torque_at_old_clutch = MAX(0, trq);
        } else if (4 == this->subphase_mod) {
#define MOD_RAMP_4_TIME 80
            // No exit (Governed by shift pressure)
            float fr_fo = pm->release_coefficient() / pm->sliding_coefficient();
            int trq_off_clutch = MAX(0, abs_input_torque - (((1.0-sid->inf.centrifugal_factor_off_clutch)*this->freeing_torque_calc) + (fr_fo*this->torque_at_new_clutch)));
            int wp_old_clutch = pm->p_clutch_with_coef(sid->curr_g, sid->releasing, trq_off_clutch, CoefficientTy::Sliding);

            p_now->off_clutch = interpolate_float(elapsed_mod, sid->ptr_prev_pressures->on_clutch, wp_old_clutch, 0, MOD_RAMP_4_TIME, InterpType::Linear);
            p_now->overlap_mod = sid->release_spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                (p_now->shift_sol_req * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
            , 0);
            if (sid->change == GearChange::_3_2) {
                p_now->mod_sol_req = MAX(
                    (p_now->shift_sol_req/2 * sid->inf.pressure_multi_spc)+
                    ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                    sid->inf.mpc_pressure_spring_reduction
                , 0);
            }
            this->torque_at_old_clutch = 0;
        }
    } else if (phase_id == PHASE_MAX_PRESSURE) {
        this->trq_ramp_up = true;
        // Max pressure phase. Pressures on the applied clutch are ramped up to ensure locking in 2 ramps.
        int wp_new_clutch = pm->p_clutch_with_coef(sid->targ_g, sid->applying, abs_input_torque, CoefficientTy::Release) + filling_adder;
        ShiftPressures* p_prev = sid->ptr_prev_pressures;
        // Clutches
        p_now->on_clutch = interpolate_float(phase_elapsed, p_prev->on_clutch, sid->SPC_MAX, 0, sid->maxp_info.ramp_time, InterpType::Linear);
        p_now->off_clutch = 0;
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->release_spring_off_clutch;
        // DIFFERS! - For mod_sol_req, use a different value for overlap_shift
        int overlap_shift_mod_sol = MIN(p_now->on_clutch, wp_new_clutch + sid->release_spring_off_clutch);
        // Solenoids (important!)
        p_now->shift_sol_req = p_now->on_clutch;
        p_now->mod_sol_req  = (
            ((overlap_shift_mod_sol - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
            (-centrifugal_force_off_clutch * sid->inf.centrifugal_factor_off_clutch) +
            sid->inf.mpc_pressure_spring_reduction
        );
        if (sid->change == GearChange::_3_2) {
            p_now->mod_sol_req = MAX(0, p_now->mod_sol_req - 500);
        }
        this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch - centrifugal_force_on_clutch - sid->release_spring_on_clutch, CoefficientTy::Sliding);
        if (phase_elapsed > sid->maxp_info.hold_time + sid->maxp_info.ramp_time) {
            // Turn off the shift circuit!
            pm->set_shift_circuit(sid->inf.shift_circuit, false);
            sid->tcc->on_shift_ending();
            ret = PHASE_END_CONTROL;
        }
        this->torque_at_old_clutch = 0;
    } else if (phase_id == PHASE_END_CONTROL) {
        // Shift solenoid is off
        int wp_gear = pm->find_working_mpc_pressure(sid->targ_g);
        p_now->on_clutch = 0;
        p_now->off_clutch = 0;
        p_now->overlap_mod = 0;
        p_now->on_clutch = sid->SPC_MAX;
        p_now->shift_sol_req = sid->SPC_MAX;
        p_now->mod_sol_req = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->mod_sol_req, wp_gear, 0, 250, InterpType::Linear);
        this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch - centrifugal_force_on_clutch - sid->release_spring_on_clutch, CoefficientTy::Sliding);
        if (phase_elapsed > 250) {
            ret = STEP_RES_END_SHIFT;
        }
        this->torque_at_old_clutch = 0;
    } else {
        ret = STEP_RES_END_SHIFT; // WTF? Should never happen
    }


    // Do torque request calculations
    bool time_for_trq_req = (sid->ptr_r_clutch_speeds->off_clutch_speed > 100) || (phase_id == PHASE_MAX_PRESSURE);
    
    int trq_req_protection = 0;
    if (phase_id == PHASE_FILL_AND_RELEASE && subphase_shift < 3 && sd->input_torque > this->torque_at_old_clutch) {
        trq_req_protection = MAX(0, sd->input_torque - this->torque_at_old_clutch);
    }

    if (trq_req_protection != 0) {
        this->set_trq_request_val(trq_req_protection);
        this->trigger_trq_request(total_elapsed);
    } else {
        this->set_trq_request_val(trq_request_raw);
    }

    // Monitor our RPMs
    if (!this->monitor_rpm && sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
        this->last_on_rpm = sid->ptr_prev_pressures->on_clutch;
        this->monitor_rpm = true;
    }
    if (this->monitor_rpm) {
        if (nullptr != this->on_clutch_delta) {
            this->on_clutch_delta->add_sample(this->last_on_rpm - sid->ptr_r_clutch_speeds->on_clutch_speed);
            this->last_on_rpm = sid->ptr_prev_pressures->on_clutch;
        }
    }

    // Calculate syncronize point
    bool sync_trigger_up = false;
    if (nullptr != this->on_clutch_delta) {
        if (ShiftHelpers::ms_till_target_on_rpm(100, this->on_clutch_delta->get_average(), sid->ptr_r_clutch_speeds->on_clutch_speed) <= 160) {
            sync_trigger_up = true;
        }
    }

    // Now check if the model is active or not (Check up ramp first)
    if (sync_trigger_up || sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm || phase_id == PHASE_MAX_PRESSURE) {
        this->disable_trq_request(total_elapsed);
    } else if (time_for_trq_req) {
        this->trigger_trq_request(total_elapsed);
    }

    int request_val = 0;
    request_val = this->get_trq_req_ramp_val(total_elapsed, 120, 120);

    if (sd->indicated_torque <= sd->min_torque || sd->converted_torque <= sd->min_torque || sd->input_rpm < 1000) {
        request_val = 0;
    }
    
    request_val = MAX(0, MIN(request_val, sd->indicated_torque)); // Ensure not out of bounds :)

    if (0 != request_val) {
        bool up_ramp = this->trq_request_is_end_ramp();
        sid->ptr_w_trq_req->amount = sd->indicated_torque - request_val;
        sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
        sid->ptr_w_trq_req->ty =  up_ramp ? TorqueRequestControlType::BackToDemandTorque : TorqueRequestControlType::NormalSpeed;
    } else {
        // No request
        sid->ptr_w_trq_req->ty = TorqueRequestControlType::None;
        sid->ptr_w_trq_req->amount = 0;
        sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
    }
    return ret;
}

uint8_t ReleasingShift::max_shift_stage_id() {
    return PHASE_END_CONTROL;
}