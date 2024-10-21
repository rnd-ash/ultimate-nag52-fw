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

uint16_t calc_trq_req(uint16_t input_rpm, uint16_t req_trq, uint16_t max_trq) {
    // Higher RPM = Higher request
    float rpm_multi = interpolate_float(input_rpm, 0.0, 0.4, 1500, 4000, InterpType::Linear);
    // Higher Trq = Higher request
    float trq_multi = interpolate_float(req_trq, 0.0, 0.4, 50, max_trq/2, InterpType::Linear);
    float r = (float)req_trq * (rpm_multi+trq_multi);
    return MIN(r, req_trq);
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
    float inertia = ShiftHelpers::calcualte_abs_engine_inertia(sid->inf.map_idx, sd->engine_rpm, sd->input_rpm);
    float drag = pm->find_turbine_drag(sid->inf.map_idx);

    // Keep calculating these values until we have to start using them
    if (phase_id == PHASE_BLEED) {
        float max = interpolate_float(sid->targ_time, 2.0, 3.0, 1000, 100, InterpType::Linear);
        this->sports_factor = interpolate_float(sd->pedal_delta->get_average(), 1.0, max, 10, 200, InterpType::Linear); //10%/sec - 200%/sec
    }
    // Threshold RPM for ramping up based on torque
    int freeing_torque = pm->find_freeing_torque(sid->change, sd->converted_torque, sd->output_rpm);
    // Calculate torque request trq here. (Only used until we ramp down torque
    //int clutch_max_trq_req = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, sid->SPC_MAX, true);
    int trq_request_raw = freeing_torque;
    if (is_upshift) {
        trq_request_raw = trq_request_raw;
        if (sid->profile == race) {
            trq_request_raw *= 1.5;
        }
    } else {
        // Downshift uses pedal multiplier
        trq_request_raw = this->sports_factor * trq_request_raw;
    }
    trq_request_raw = MIN(trq_request_raw, abs_input_torque);
    freeing_torque = MIN(this->sports_factor * freeing_torque, abs_input_torque);
    int effective_torque = MIN(freeing_torque,
                                (freeing_torque + this->torque_at_new_clutch) / 2);

    this->threshold_rpm =
            (effective_torque + this->torque_at_new_clutch) *
            ((80.0 + 40.0) / 1000.0) *
            // 80 for MPC ramp time, 20*2 (40) for computation delay over CAN (Rx of Sta. Trq -> Tx of EGS Trq)
            drag /
            inertia;
    this->threshold_rpm = MAX(this->threshold_rpm, 100);
    //}

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
        this->torque_at_old_clutch = abs_input_torque;
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
#define RAMP_TIME 80
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
#define HOLD_3_TIME 40
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
            
            bool sync_with_mod = phase_elapsed > (this->mod_time_phase_0 + this->mod_time_phase_1);
            // Sync with mod reached, but no speed difference
            // Or timeout
            // Or new clutch speed is near target
            if ((sid->ptr_r_clutch_speeds->off_clutch_speed < 100 && sync_with_mod) || elapsed > 5000 || sid->ptr_r_clutch_speeds->on_clutch_speed < 100) {
                this->subphase_shift = 4;
                this->ts_phase_shift = phase_elapsed;
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, false);
        } else if (4 == this->subphase_shift) { // Ramping new clutch (Clutch is still not moving)
            uint16_t elapsed = phase_elapsed - this->ts_phase_shift;
            this->filling_adder += 8.0;
            this->filling_adder += (this->sports_factor * 0.05 * this->filling_adder);
            p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch + filling_adder;
            p_now->overlap_shift = sid->spring_on_clutch + p_now->on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm || elapsed > 5000) {
                this->subphase_shift = 5;
                this->ts_phase_shift = phase_elapsed;
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, false);
        } else if (5 == this->subphase_shift) { // Holding upper pressure
            uint16_t elapsed = phase_elapsed - this->ts_phase_shift;
            p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch + filling_adder;
            p_now->overlap_shift = sid->spring_on_clutch + p_now->on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (sid->ptr_r_clutch_speeds->on_clutch_speed < 100 || elapsed > 5000) {
                // END FILLING AND RELEASE
                if (this->torque_req_en) {
                    // Jump if we have issued a torque request to MAX pressure
                    // This makes dynamic shifts a lot faster
                    ret = PHASE_MAX_PRESSURE;
                } else {
                    ret = PHASE_OVERLAP;
                }
            }
            this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, false);
        } else {
            ret = PHASE_OVERLAP; // WTF
        }
        // Mod pressure
        if (0 == this->subphase_mod) {
            uint16_t elapsed = phase_elapsed - this->ts_phase_mod;
            int max_time = sid->prefill_info.fill_time + RAMP_TIME + HOLD_3_TIME;
            if (sid->change == ProfileGearChange::TWO_ONE || sid->change == ProfileGearChange::THREE_TWO) {
                max_time = 0;
            } else {
                int reduction = 0;
                if (0 != freeing_torque) {
                    reduction = inertia * (float)(sid->ptr_r_clutch_speeds->on_clutch_speed) / drag / (float)freeing_torque;
                }
                max_time = MAX(max_time, max_time - reduction);
            }
            this->mod_time_phase_0 = max_time;

            int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, MAX(abs_input_torque, 30));

            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            if (elapsed > this->mod_time_phase_0 || sid->ptr_r_clutch_speeds->on_clutch_speed < 100) {
                this->subphase_mod = 1;
                this->ts_phase_mod = phase_elapsed;
            }
            this->torque_at_old_clutch = MAX(abs_input_torque, 30);
        } else if (1 == this->subphase_mod) {
            uint16_t elapsed = phase_elapsed - this->ts_phase_mod;
            if (this->mod_time_phase_1 == -1) {
                #define MOD_RAMP_TIME 80
                // Initialize ramp time
                this->mod_time_phase_1 = MAX(   
                    MOD_RAMP_TIME,
                    sid->prefill_info.fill_time + RAMP_TIME - this->mod_time_phase_0
                );
                if (sid->change == ProfileGearChange::TWO_ONE || sid->change == ProfileGearChange::THREE_TWO) {
                    this->mod_time_phase_1 = MOD_RAMP_TIME;
                }
            }

            int wp_old_clutch = pm->find_working_pressure_for_clutch(sid->curr_g, sid->releasing, MAX(30, abs_input_torque - freeing_torque), false);
            p_now->off_clutch = interpolate_float(elapsed, sid->ptr_prev_pressures->off_clutch, wp_old_clutch, 0, this->mod_time_phase_1, InterpType::Linear);
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            if (
                elapsed > this->mod_time_phase_1 ||
                    (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm &&
                    sid->ptr_r_clutch_speeds->off_clutch_speed > 100)
                || sid->ptr_r_clutch_speeds->on_clutch_speed < 100
            ) {
                this->subphase_mod = 2;
                this->ts_phase_mod = phase_elapsed;
            }
            this->torque_at_old_clutch = pm->calc_max_torque_for_clutch(sid->curr_g, sid->releasing,  p_now->off_clutch, false);
        } else if (2 == this->subphase_mod) {
            // Reducing until releasing the clutch
            this->filling_trq_reducer += (10.0 * this->sports_factor) + (this->sports_factor * 0.05 * this->filling_trq_reducer);
            int trq = MAX(0, abs_input_torque - freeing_torque - filling_trq_reducer);
            int wp_old_clutch = 0;
            wp_old_clutch = pm->find_working_pressure_for_clutch(sid->curr_g, sid->releasing, trq, false);
            
            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
                , 0);
            if (
                trq <= 0 || // No more torque to reduce by
                (sid->ptr_r_clutch_speeds->off_clutch_speed > 100) || // Released old clutch
                (sid->ptr_r_clutch_speeds->on_clutch_speed < 100) // Early sync.
            ) {
                this->subphase_mod = 3;
                this->ts_phase_mod = phase_elapsed;
            }
            this->torque_at_old_clutch = pm->calc_max_torque_for_clutch(sid->curr_g, sid->releasing,  p_now->off_clutch, false);
        } else if (3 == this->subphase_mod) {
            // Hold pressure until we can sync. 
            int trq = MAX(0, abs_input_torque - freeing_torque - filling_trq_reducer);
            int wp_old_clutch = pm->find_working_pressure_for_clutch(sid->curr_g, sid->releasing, trq, false);
            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
            , 0);
            // Crossover point, start sync.
            if (
                (sid->ptr_r_clutch_speeds->on_clutch_speed < this->threshold_rpm) // Begin merging of clutches
            ) {
                this->subphase_mod = 4;
                this->ts_phase_mod = phase_elapsed;
            }
            this->torque_at_old_clutch = MAX(0, trq);
        } else if (4 == this->subphase_mod) {
#define MOD_RAMP_4_TIME 80
            uint16_t elapsed = phase_elapsed - this->ts_phase_mod;
            // No exit (Governed by shift pressure)
            float fr_fo = pm->release_coefficient() / pm->friction_coefficient();
            int trq_off_clutch = MAX(0, abs_input_torque - (((1.0-sid->inf.centrifugal_factor_off_clutch)*freeing_torque) + (fr_fo*this->torque_at_new_clutch)));
            int wp_old_clutch = pm->find_working_pressure_for_clutch(sid->curr_g, sid->releasing, trq_off_clutch, false);

            p_now->off_clutch = interpolate_float(elapsed, sid->ptr_prev_pressures->on_clutch, wp_old_clutch, 0, MOD_RAMP_4_TIME, InterpType::Linear);
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = MAX(
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
                sid->inf.mpc_pressure_spring_reduction
            , 0);
            this->torque_at_old_clutch = 0;
        }
    } else if (phase_id == PHASE_OVERLAP) {
        int duration = 140; // FROM EGS CAL
        // New clutch gets full pressure (+momentum)
        // Old clutch valve is empties completely
        int old_new_clutch = sid->ptr_prev_pressures->on_clutch;
        int wp_new_clutch_weak = pm->find_releasing_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque);
        p_now->on_clutch = MAX(wp_new_clutch_weak, old_new_clutch);
        this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, false);

        p_now->off_clutch = 0;
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;

        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
        
        if (sid->ptr_r_clutch_speeds->on_clutch_speed < 100) {
            ret = PHASE_MAX_PRESSURE;
        }
    } else if (phase_id == PHASE_MAX_PRESSURE) {
        this->trq_ramp_up = true;
        // Max pressure phase. Pressures on the applied clutch are ramped up to ensure locking in 2 ramps.
        int wp_new_clutch = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque, false) + filling_adder;
        ShiftPressures* p_prev = sid->ptr_prev_pressures;
        // Clutches
        p_now->on_clutch = interpolate_float(phase_elapsed, p_prev->on_clutch, MAX(wp_new_clutch, p_prev->on_clutch), 0, sid->maxp_info.ramp_time, InterpType::Linear);
        p_now->off_clutch = 0;
        // Overlap valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = interpolate_float(phase_elapsed, p_prev->overlap_shift, sid->SPC_MAX, 0, sid->maxp_info.ramp_time, InterpType::Linear);
        // DIFFERS! - For mod_sol_req, use a different value for overlap_shift
        int overlap_shift_mod_sol = MIN(p_now->overlap_shift, wp_new_clutch + sid->spring_on_clutch);
        // Solenoids (important!)
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req  = (
            ((overlap_shift_mod_sol - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
            (-centrifugal_force_off_clutch * sid->inf.centrifugal_factor_off_clutch) +
            sid->inf.mpc_pressure_spring_reduction
        );
        this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->overlap_shift - centrifugal_force_on_clutch - sid->spring_on_clutch, false);
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
        p_now->overlap_shift = 0;
        p_now->shift_sol_req = sid->SPC_MAX;
        p_now->mod_sol_req = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->mod_sol_req, wp_gear, 0, 250, InterpType::Linear);
        this->torque_at_new_clutch = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->overlap_shift - centrifugal_force_on_clutch - sid->spring_on_clutch, false);
        if (phase_elapsed > 250) {
            ret = STEP_RES_END_SHIFT;
        }
        this->torque_at_old_clutch = 0;
    } else {
        ret = STEP_RES_END_SHIFT; // WTF? Should never happen
    }

    bool time_for_trq_req = (phase_id == PHASE_FILL_AND_RELEASE && subphase_shift >= 3) || (phase_id >= PHASE_OVERLAP && phase_id <= PHASE_MAX_PRESSURE);
    int trq_req_now = 0;
    bool trq_ramp_inc = false;
    // We can only request if engine torque is above engines min torque
    if (sd->indicated_torque > sd->min_torque && sd->converted_torque > sd->min_torque && sd->input_rpm > 1000 && this->trq_req) {
        if (phase_id == PHASE_FILL_AND_RELEASE && subphase_shift < 3 && sd->input_torque > this->torque_at_old_clutch) {
            trq_req_now = MAX(0, sd->input_torque - this->torque_at_old_clutch);
        } else if (time_for_trq_req) {
            if (sd->input_torque >= 500) {
                trq_req_now = MAX(0, MAX(sd->input_torque - 500, trq_request_raw));
            } else {
                trq_req_now = MAX(0, trq_request_raw);
            }
        }
    }
    bool set_last_stage = true;
    if (sd->indicated_torque < 40) { // (TODO - Configure this param)
        trq_req_now = 0; // Disable intervension for low torque
    }
    int trq_req_output = this->trq_req_last_ramp;
    if (!this->torque_req_en) {
        // Checks to when torque request should begin
        if (phase_id == PHASE_FILL_AND_RELEASE) {
            if (subphase_mod == 0) {
                if (sd->input_torque > this->torque_at_old_clutch) {
                    this->torque_req_en = true;
                }
            } else if (subphase_mod == 1) {
                if (sd->input_torque > this->torque_at_old_clutch) {
                    this->torque_req_en = true;
                }
            } else if (subphase_shift >= 3) {
                // Ramping or new
                if (
                    (sid->ptr_r_clutch_speeds->on_clutch_speed <= this->threshold_rpm && sid->ptr_r_clutch_speeds->off_clutch_speed > 100) || 
                    sid->ptr_r_clutch_speeds->on_clutch_speed < 100) 
                {
                    this->torque_req_en = true;
                }
            }
        }
    } 
    
    if (this->torque_req_en) {
        if (phase_id < PHASE_MAX_PRESSURE) {
            // Start ramp to intercept
            // 60ms from EGS52 cali
            if (this->trq_ramp_down_time == 0) {
                this->trq_ramp_down_time = total_elapsed;
            }
            int into = total_elapsed - this->trq_ramp_down_time;
            trq_req_output = interpolate_float(into, 0, trq_req_now, 0, 60, InterpType::Linear);
        } else {
            // Ramp back up
            if (this->trq_ramp_up_time == 0) {
                this->trq_ramp_up_time = total_elapsed;
            }
            int into = total_elapsed - this->trq_ramp_up_time;
            trq_req_output = interpolate_float(into, this->trq_req_last_ramp, 0, 0, 160, InterpType::Linear);
            if (into > 160) {
                this->torque_req_en = false; // Disable latch
            }
            set_last_stage = false;
        }
    }
    trq_req_output = MAX(0, MIN(trq_req_output, sd->indicated_torque));
    
    // We have our target (trq_req_now)
    // So now we smooth it based on the stage of shift. This gives us smooth ramps
    if (set_last_stage) {
        this->trq_req_last_ramp = trq_req_output;
    }
    if (this->torque_req_en && trq_req_output != 0) {
        sid->ptr_w_trq_req->amount = sd->indicated_torque - trq_req_output;
        sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
        sid->ptr_w_trq_req->ty =  this->trq_ramp_up ? TorqueRequestControlType::BackToDemandTorque : TorqueRequestControlType::NormalSpeed;
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