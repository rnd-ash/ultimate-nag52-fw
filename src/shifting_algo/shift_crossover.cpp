#include "shift_crossover.h"

const uint8_t PHASE_BLEED     = 0;
const uint8_t PHASE_PREFILL   = 1;

const uint8_t PHASE_OVERLAP = 2;

const uint8_t PHASE_MOMENTUM_CONTROL = 3;
const uint8_t PHASE_MAX_PRESSURE  = 4;
const uint8_t PHASE_END_CONTROL   = 5;

const uint8_t FILL_RAMP_TIME = 100;
const uint8_t FILL_HOLD_TIME = 100;

CrossoverShift::CrossoverShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {}
CrossoverShift::~CrossoverShift() {}

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
    float drag = pm->find_turbine_drag(sid->inf.map_idx);
    this->inertia = ShiftHelpers::calcualte_abs_engine_inertia(sid->inf.map_idx, sd->engine_rpm, sd->input_rpm);
    
    int decent_adder_torque = pm->find_decent_adder_torque(sid->change, abs(sd->converted_driver_torque), sd->output_rpm);
    this->decent_adder_torque = decent_adder_torque;
    float min_holding = MIN(
        MAX(
            ((pm->friction_coefficient() / pm->release_coefficient()) - 1.0) * abs_input_torque,
            50.0
        ),
        decent_adder_torque
    );
    
    this->threshold_rpm =
                (decent_adder_torque + min_holding) *
                ((40.0 + 40.0) / 1000.0) *
                // 80 for MPC ramp time, 20*2 (40) for computation delay over CAN (Rx of Sta. Trq -> Tx of EGS Trq)
                drag /
                inertia;
    this->threshold_rpm = MAX(100, threshold_rpm);
    
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
            ret = PHASE_PREFILL;
        }
    } else if (phase_id == PHASE_PREFILL) {
        int elapsed_shift = this->elapsed_subphase_shift(phase_elapsed);
        int elapsed_mod = this->elapsed_subphase_mod(phase_elapsed);
        // This check is ran ONCE on shift initialization at the start of the filling phase
        if (!filling_mode_check) {
            filling_mode_check = true;
            if (abs_input_torque <= VEHICLE_CONFIG.engine_drag_torque/10.0) {
                this->do_high_filling = false;
                // Low mode (Special)
                this->subphase_mod = 0; // 0 -> 1 -> Done (Hold, drop)
            } else {
                // High mode (Constant)
                this->do_high_filling = true;
                this->subphase_mod = 2; // 2 -> Done (Hold to keep clutch off)
            }
        }

        // On clutch control
        if (0 == this->subphase_shift) {
            p_now->on_clutch = sid->prefill_info.fill_pressure_on_clutch;
            p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (elapsed_shift > sid->prefill_info.fill_time || sid->ptr_r_clutch_speeds->on_clutch_speed > 100) {
                this->inc_subphase_shift(phase_elapsed);
            }
        } else if (1 == this->subphase_shift) { // Filling ramp
            if (this->do_high_filling) {
                p_now->on_clutch = interpolate_float(subphase_shift, sid->ptr_prev_pressures->on_clutch, sid->prefill_info.low_fill_pressure_on_clutch, 0, FILL_RAMP_TIME, InterpType::Linear);
            } else {
                // Drop to 0 pressure when filling with little torque
                p_now->on_clutch = interpolate_float(subphase_shift, sid->ptr_prev_pressures->on_clutch, 0, 0, FILL_RAMP_TIME, InterpType::Linear);
            }
            p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (elapsed_shift > FILL_RAMP_TIME || sid->ptr_r_clutch_speeds->on_clutch_speed > 100) {
                this->inc_subphase_shift(phase_elapsed);
            }
        } else if (2 == this->subphase_shift) { // Lower to 0 pressure
            if (this->do_high_filling) {
                p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch;
                p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
                p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
                // Exit condition - Check clutch is moving
                if (elapsed_shift > FILL_HOLD_TIME || sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
                    this->min_spc_clutch_allowed = p_now->on_clutch;
                    ret = PHASE_OVERLAP;
                }
            } else {
                // 2 stage ramp (Hacky way of doing it on 1 phase)
#define FILL_RAMP_P_1 500
#define FILL_RAMP_P_2 700
#define FILL_RAMP_T_1 500
#define FILL_RAMP_T_2 240
                if (elapsed_shift <= FILL_RAMP_T_1) {
                    p_now->on_clutch = interpolate_float(subphase_shift, 0, FILL_RAMP_P_1, 0, FILL_RAMP_T_1, InterpType::Linear);
                } else {
                    int t = elapsed_shift - FILL_RAMP_T_1;
                    p_now->on_clutch = interpolate_float(t, FILL_RAMP_P_1, FILL_RAMP_P_2, 0, FILL_RAMP_T_2, InterpType::Linear);
                }
                p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
                p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
                if (elapsed_shift > FILL_RAMP_T_1 + FILL_RAMP_T_2 || sid->ptr_r_clutch_speeds->on_clutch_speed < 100 || abs_input_torque > (VEHICLE_CONFIG.engine_drag_torque/10.0 * 1.5)) {
                    this->min_spc_clutch_allowed = p_now->on_clutch;
                    ret = PHASE_OVERLAP;
                }
            }
        } else {}


        // LOW MOD MODE (LOW PRESSURE FILLING)
        if (0 == this->subphase_mod) { // Low mode phase 1
            int min_torque = MAX(30, abs_input_torque);
            int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, min_torque);
            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = (
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((p_now->overlap_mod -centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                sid->inf.mpc_pressure_spring_reduction
            );

            // Now we have to calc the pressure for overlap phase so mod pressure doesn't drop below it
            // Floor target SPC pressure to low filling pressure
            int spc_target = MAX(
                MAX(pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque, false), sid->prefill_info.low_fill_pressure_on_clutch),
                sid->prefill_info.low_fill_pressure_on_clutch
            ) + sid->spring_on_clutch;

            int mod_min = ( // Assumes p_mod->off_clutch = 0 (No mod pressure at the clutch)
                ((spc_target - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((sid->spring_off_clutch - centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                sid->inf.mpc_pressure_spring_reduction
            );
            p_now->mod_sol_req = MAX(p_now->mod_sol_req, mod_min);

            if (elapsed_mod > sid->prefill_info.fill_time) {
                // Next phase
                this->inc_subphase_mod(phase_elapsed);
            }
        } else if (1 == this->subphase_mod) { // Low mode phase 2
            // Does not end, waits for the end of shift pressure ramp
            p_now->off_clutch = 0;
            p_now->overlap_mod = 0; // Completely empty out the modulating side of the overlap valve
            p_now->mod_sol_req = (
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((p_now->overlap_mod -centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                sid->inf.mpc_pressure_spring_reduction
            );
        } 
        // HIGH MOD MODE (HIGH PRESSURE FILLING)
        else if (2 == this->subphase_mod) {
            // Does not end, waits for the end of shift pressure ramp
            int min_torque = MAX(30, abs_input_torque);
            int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, min_torque);
            p_now->off_clutch = wp_old_clutch;
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = (
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((p_now->overlap_mod -centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                sid->inf.mpc_pressure_spring_reduction
            );

            // Now we have to calc the pressure for overlap phase so mod pressure doesn't drop below it
            int spc_target = MAX(
                pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque, false),
                sid->prefill_info.low_fill_pressure_on_clutch
            ) + sid->spring_on_clutch;

            int mod_min = ( // Assumes p_mod->off_clutch = 0 (No mod pressure at the clutch)
                ((spc_target - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((sid->spring_off_clutch - centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                sid->inf.mpc_pressure_spring_reduction
            );
            p_now->mod_sol_req = MAX(p_now->mod_sol_req, mod_min);
        } else {}
    } else if (phase_id == PHASE_OVERLAP) {
#define OVERLAP_TIME 500
        // SHIFT PRESSURE
        int torque = abs_input_torque;
        int previous_clutch_spc = this->min_spc_clutch_allowed;
        int spc_at_end = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, torque, false);

        int spc_now = interpolate_float(phase_elapsed, previous_clutch_spc, spc_at_end, 0, OVERLAP_TIME, InterpType::Linear);
        p_now->on_clutch = MAX(previous_clutch_spc, spc_now);
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;

        // MOD PRESSURE
        if (this->do_high_filling) {
            int trq = MAX(0, abs_input_torque - pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, p_now->on_clutch, false)); // Torque left
            p_now->off_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, trq);
            p_now->overlap_mod = sid->spring_off_clutch + p_now->off_clutch;
            p_now->mod_sol_req = (
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                sid->inf.mpc_pressure_spring_reduction
            );
            int spc_min = previous_clutch_spc + sid->spring_on_clutch;
            int mod_min = (
                ((spc_min - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((sid->spring_off_clutch - centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                sid->inf.mpc_pressure_spring_reduction
            );
            p_now->mod_sol_req = MAX(p_now->mod_sol_req, mod_min);
        } else {
            // Low pressure filling (Constant)
            p_now->off_clutch = 0;
            p_now->overlap_mod = 0;
            p_now->mod_sol_req = (
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                sid->inf.mpc_pressure_spring_reduction
            );
        }

        // Next phase once clutch is moving, or timeout for phase
        if (phase_elapsed > OVERLAP_TIME || sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
            ret = PHASE_MOMENTUM_CONTROL;
            trq_req_start_time = total_elapsed;
        }
    } else if (phase_id == PHASE_MOMENTUM_CONTROL) {
        int elapsed_shift = this->elapsed_subphase_shift(phase_elapsed);
        int elapsed_mod = this->elapsed_subphase_mod(phase_elapsed);
        if (0 == this->subphase_shift) { // Add in inertia to shift pressure via a ramp
            int trq = abs_input_torque + this->adder_torque;
            int on_clutch_start = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq, false);
            int on_clutch_end = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq + decent_adder_torque, false);

            p_now->on_clutch = MAX(interpolate_float(elapsed_shift, on_clutch_start, on_clutch_end, 0, 180, InterpType::Linear), this->min_spc_clutch_allowed);
            p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (elapsed_shift > 180 || sid->ptr_r_clutch_speeds->on_clutch_speed < threshold_rpm) {
                // Next phase
                this->inc_subphase_shift(phase_elapsed);
            }
            this->adder_torque += 1;
        } else if (1 == this->subphase_shift) { // Hold the new pressure until the clutch moves past syncronize threshold speed.
            int trq = abs_input_torque + this->adder_torque + this->decent_adder_torque;
            int on_clutch_end = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq, false);
            p_now->on_clutch = MAX(on_clutch_end, this->min_spc_clutch_allowed);
            p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (sid->ptr_r_clutch_speeds->on_clutch_speed < threshold_rpm || elapsed_shift > 5000) { // Only finish when clutch is moved
                // Next phase
                this->inc_subphase_shift(phase_elapsed);
            }
        } else if (2 == this->subphase_shift) { // Reduce pressure slightly to avoid a clunk at the end of the clutch movement
            int trq = abs_input_torque + this->adder_torque + this->decent_adder_torque;
            int trq_end = abs_input_torque + this->adder_torque + this->decent_adder_torque;//min_holding;
            int on_clutch_start = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq, false);
            int on_clutch_end = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq_end, false);
            p_now->on_clutch = MAX(interpolate_float(elapsed_shift, on_clutch_start, on_clutch_end, 0, 40, InterpType::Linear), this->min_spc_clutch_allowed);
            p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if ((sid->ptr_r_clutch_speeds->on_clutch_speed < 100 && elapsed_shift > 40) || elapsed_shift > 1000) { // Clutch moved or timeout
                this->inc_subphase_shift(phase_elapsed);
            }
        } else if (3 == this->subphase_shift) { // Hold for a few ms to let pressures settle
            // Hold pressures as they were
            p_now->on_clutch = sid->ptr_prev_pressures->on_clutch;
            p_now->overlap_shift = sid->ptr_prev_pressures->overlap_shift;
            p_now->shift_sol_req = sid->ptr_prev_pressures->shift_sol_req;
            if (elapsed_shift > 40) {
                // END OF PHASE
                ret = PHASE_MAX_PRESSURE;
            }
        } else {}
        // MOD CONTROL
        if (0 == this->subphase_mod) {
            p_now->off_clutch = 0;
            p_now->overlap_mod = 0;
            int mod_end = (
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((p_now->overlap_mod-centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                (-(sid->inf.pressure_multi_mpc*150)) + // 150 comes from calibration pointer on E55 coding (TODO - Locate in calibration and assign in CAL structure)
                sid->inf.mpc_pressure_spring_reduction
            );

            p_now->mod_sol_req  = interpolate_float(elapsed_mod, sid->ptr_prev_pressures->mod_sol_req, mod_end, 0, 40, InterpType::Linear);
            if (elapsed_mod > 40) {
                this->inc_subphase_mod(phase_elapsed);
            }
        } else if (1 == this->subphase_mod) {
            // NO end , waits for shift pressure to end the phase
            p_now->off_clutch = 0;
            p_now->overlap_mod = 0;
            p_now->mod_sol_req  = (
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((p_now->overlap_mod-centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                (-(sid->inf.pressure_multi_mpc*150)) + // 150 comes from calibration pointer on E55 coding (TODO - Locate in calibration and assign in CAL structure)
                sid->inf.mpc_pressure_spring_reduction
            );
        }

    } else if (phase_id == PHASE_MAX_PRESSURE) {
        // Max pressure phase. Pressures on the applied clutch are ramped up to ensure locking in 2 ramps.
        int wp_new_clutch = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque, false);
        ShiftPressures* p_prev = sid->ptr_prev_pressures;
        // Clutches
        p_now->on_clutch = interpolate_float(phase_elapsed, p_prev->on_clutch, MAX(wp_new_clutch, p_prev->on_clutch), 0, sid->maxp_info.ramp_time, InterpType::Linear);
        p_now->off_clutch = 0;
        // Overlap valves
        p_now->overlap_mod = 0;
        p_now->overlap_shift = interpolate_float(phase_elapsed, p_prev->overlap_shift, sid->SPC_MAX, 0, sid->maxp_info.ramp_time, InterpType::Linear);
        // DIFFERS! - For mod_sol_req, use a different value for overlap_shift
        int overlap_shift_mod_sol = MIN(p_now->overlap_shift, wp_new_clutch + sid->spring_on_clutch);
        // Solenoids (important!)
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req  = (
            ((overlap_shift_mod_sol - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
            (p_now->overlap_mod-centrifugal_force_off_clutch * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
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
    int trq_req_targ = 0;
    // We can only request if engine torque is above engines min torque
    if (sd->indicated_torque > sd->min_torque && sd->converted_torque > sd->min_torque && sd->input_rpm > 1000) {
        trq_req_targ = abs_input_torque/2;
    }
    if (trq_req_targ < 40) {
        trq_req_targ = 0;
    }

    // Now actuate (Tests for when to start the request)
    if (!this->torque_req_en) {
        if (abs(sid->ptr_r_clutch_speeds->off_clutch_speed) > 100) {
            this->torque_req_en = true;
        } else if (phase_id == PHASE_OVERLAP) {
            this->torque_req_en = true;
        }
    } 
    bool set_last_stage = true;
    int trq_req_output = 0;
    if (this->torque_req_en) {
        // We are doing the request
        if (phase_id < PHASE_MAX_PRESSURE) {
            // Start ramp to intercept
            // 60ms from EGS52 cali
            if (this->trq_ramp_down_time == 0) {
                this->trq_ramp_down_time = total_elapsed;
            }
            int into = total_elapsed - this->trq_ramp_down_time;
            trq_req_output = interpolate_float(into, 0, trq_req_targ, 0, 60, InterpType::Linear);
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
        sid->ptr_w_trq_req->ty =  phase_id >= PHASE_MAX_PRESSURE ? TorqueRequestControlType::BackToDemandTorque : TorqueRequestControlType::NormalSpeed;
    } else {
        // No request
        sid->ptr_w_trq_req->ty = TorqueRequestControlType::None;
        sid->ptr_w_trq_req->amount = 0;
        sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
    }
        

    return ret;
}

uint8_t CrossoverShift::max_shift_stage_id() {
    return PHASE_END_CONTROL;
}