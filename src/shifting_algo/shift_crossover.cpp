#include "shift_crossover.h"

const uint8_t PHASE_BLEED     = 0;
const uint8_t PHASE_PREFILL   = 1;

const uint8_t PHASE_OVERLAP = 2;

const uint8_t PHASE_MOMENTUM_CONTROL = 3;
const uint8_t PHASE_MAX_PRESSURE  = 4;
const uint8_t PHASE_END_CONTROL   = 5;

const uint8_t FILL_RAMP_TIME = 100;
const uint8_t FILL_HOLD_TIME = 100;

CrossoverShift::CrossoverShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {
    this->on_clutch_delta = new FirstOrderAverage(5);
}
CrossoverShift::~CrossoverShift() {
    delete this->on_clutch_delta;
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
    float drag = pm->find_turbine_drag(sid->inf.map_idx);
    this->inertia = ShiftHelpers::calcualte_abs_engine_inertia(sid->inf.map_idx, sd->engine_rpm, sd->input_rpm);
    int decent_adder_torque = pm->find_decent_adder_torque(sid->change, abs(sd->converted_driver_torque), sd->output_rpm);

    this->decent_adder_torque = decent_adder_torque;
    float min_holding = MIN(
        MAX(
            ((pm->applying_coefficient() / pm->release_coefficient()) - 1.0) * abs_input_torque,
            50.0
        ),
        decent_adder_torque
    );

    
    this->threshold_rpm =
                (0.5*(decent_adder_torque + min_holding) + 0.5*(MAX(0, sd->converted_driver_torque))) *
                ((40.0 + 40.0) / 1000.0) *
                // 80 for MPC ramp time, 20*2 (40) for computation delay over CAN (Rx of Sta. Trq -> Tx of EGS Trq)
                drag /
                inertia;
    this->threshold_rpm = MAX(100, threshold_rpm);
    
    if (phase_id == PHASE_BLEED) {
        float max = interpolate_float(sid->targ_time, 2.0, 3.0, 1000, 100, InterpType::Linear);
        this->sports_factor = interpolate_float(sd->pedal_delta->get_average(), 1.0, max, 10, 200, InterpType::Linear); //10%/sec - 200%/sec
    }

    int trq_request_raw = decent_adder_torque;
        if (is_upshift) {
            trq_request_raw = trq_request_raw;
        } else {
            // Downshift uses pedal multiplier
            trq_request_raw = this->sports_factor * trq_request_raw;
        }
        trq_request_raw = MIN(trq_request_raw, abs_input_torque);
        decent_adder_torque = MIN(this->sports_factor * decent_adder_torque, abs_input_torque);
        this->torque_request_calc = trq_request_raw;



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
            ret = PHASE_PREFILL;
        }
    } else if (phase_id == PHASE_PREFILL) {
        int elapsed_shift = this->elapsed_subphase_shift(phase_elapsed);
        int elapsed_mod = this->elapsed_subphase_mod(phase_elapsed);
        // This check is ran ONCE on shift initialization at the start of the filling phase
        if (!filling_mode_check) {
            filling_mode_check = true;
            this->max_torque_prefill = 0;
            if (sid->inf.map_idx != 3 && sid->inf.map_idx != 6 && sid->inf.map_idx != 7) {
                // 0 is for 4-5 and 3-2 and 2-1
                this->max_torque_prefill = 30;
            }
            if (abs_input_torque <= max_torque_prefill) {
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
                // Decrease pressure but not by too much!
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
            if (this->do_high_filling) { // High filling - we keep the pressure quite high, the off clutch is held to prevent accidental engagmenet (See below)
                p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch;
                p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
                p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
                // Exit condition - Check clutch is moving
                if (elapsed_shift > FILL_HOLD_TIME || sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
                    this->min_spc_clutch_allowed = p_now->on_clutch;
                    ret = PHASE_OVERLAP;
                }
            } else { // Low pressure filling - Ramp up from no pressure with 2 small ramps. The off clutch has been released here so we can do this as there is no torque
                // 2 stage ramp (Hacky way of doing it in 1 phase)
#define FILL_RAMP_P_1 500 // Pressure points // FROM EGS Calibrations for my E55 - TODO - Pull these into the general calibrations and extract from flash :)
#define FILL_RAMP_P_2 700
#define FILL_RAMP_T_1 500 // Duration points
#define FILL_RAMP_T_2 240
                if (elapsed_shift <= FILL_RAMP_T_1) {
                    p_now->on_clutch = interpolate_float(subphase_shift, 0, FILL_RAMP_P_1, 0, FILL_RAMP_T_1, InterpType::Linear);
                } else {
                    int t = elapsed_shift - FILL_RAMP_T_1;
                    p_now->on_clutch = interpolate_float(t, FILL_RAMP_P_1, FILL_RAMP_P_2, 0, FILL_RAMP_T_2, InterpType::Linear);
                }
                p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
                p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
                if (elapsed_shift > FILL_RAMP_T_1 + FILL_RAMP_T_2 || sid->ptr_r_clutch_speeds->on_clutch_speed < 100 || abs_input_torque > max_torque_prefill * 1.5) {
                    this->min_spc_clutch_allowed = p_now->on_clutch;
                    ret = PHASE_OVERLAP;
                }
            }
        } else {}


        // OFF CLUTCH CONTROL FOR FILLING PHASE
        if (0 == this->subphase_mod) { // Low pressure mode phase 1
            int min_torque = MAX(50, abs_input_torque);
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
                // Next step of the phase
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
        } else if (2 == this->subphase_mod) { // high pressure filling
            // Does not end, waits for the end of shift pressure ramp
            int min_torque = MAX(50, abs_input_torque);
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
        int overlap_time = interpolate_float(abs_input_torque, 180, 100, 100, 400, InterpType::Linear);
        // SHIFT PRESSURE
        int torque = abs_input_torque;
        int previous_clutch_spc = this->min_spc_clutch_allowed;
        int spc_at_end = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, torque, false);

        int spc_now = interpolate_float(phase_elapsed, previous_clutch_spc, spc_at_end, 0, overlap_time, InterpType::Linear);
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
        if (phase_elapsed > overlap_time || sid->ptr_r_clutch_speeds->off_clutch_speed > 100) {
            ret = PHASE_MOMENTUM_CONTROL;
        }
    } else if (phase_id == PHASE_MOMENTUM_CONTROL) {
        int overlap_time = interpolate_float(abs_input_torque, 180, 100, 100, 400, InterpType::Linear);
        int elapsed_shift = this->elapsed_subphase_shift(phase_elapsed);
        int elapsed_mod = this->elapsed_subphase_mod(phase_elapsed);
        if (0 == this->subphase_shift) { // Add in inertia to shift pressure via a ramp
            int trq = abs_input_torque + this->adder_torque;
            int on_clutch_start = sid->ptr_prev_pressures->on_clutch;
            int on_clutch_end = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq + decent_adder_torque, false);

            p_now->on_clutch = MAX(interpolate_float(elapsed_shift, on_clutch_start, on_clutch_end, 0, overlap_time, InterpType::Linear), this->min_spc_clutch_allowed);
            p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (elapsed_shift > overlap_time || sid->ptr_r_clutch_speeds->on_clutch_speed < threshold_rpm) {
                // Next phase
                this->inc_subphase_shift(phase_elapsed);
            }
        } else if (1 == this->subphase_shift) { // Hold the new pressure until the clutch moves past syncronize threshold speed.
            int trq = abs_input_torque + this->adder_torque + this->decent_adder_torque + sid->ptr_w_trq_req->amount;
            int on_clutch_end = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq, false);
            p_now->on_clutch = MAX(on_clutch_end, this->min_spc_clutch_allowed);
            p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if (sid->ptr_r_clutch_speeds->on_clutch_speed < threshold_rpm || elapsed_shift > 5000) { // Only finish when clutch is moved
                // Next phase
                this->inc_subphase_shift(phase_elapsed);
            }
            this->adder_torque += interpolate_float(sid->targ_time, 1.0, 3.0, 100, 500, InterpType::Linear);
        } else if (2 == this->subphase_shift) { // Reduce pressure slightly to avoid a clunk at the end of the clutch movement
            int trq = abs_input_torque + this->adder_torque + this->decent_adder_torque + sid->ptr_w_trq_req->amount;
            int trq_end = abs_input_torque + this->adder_torque + this->decent_adder_torque + sid->ptr_w_trq_req->amount;
            int on_clutch_start = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq, false);
            int on_clutch_end = pm->find_working_pressure_for_clutch(sid->targ_g, sid->applying, trq_end, false);
            p_now->on_clutch = MAX(interpolate_float(elapsed_shift, on_clutch_start, on_clutch_end, 0, 60, InterpType::Linear), this->min_spc_clutch_allowed);
            p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
            p_now->shift_sol_req = p_now->overlap_shift - centrifugal_force_on_clutch;
            if ((sid->ptr_r_clutch_speeds->on_clutch_speed < 100 && elapsed_shift > 60) || elapsed_shift > 1000) { // Clutch moved or timeout
                // END OF PHASE
                ret = PHASE_MAX_PRESSURE;
            }
        } else {}
        // MOD CONTROL
        if (0 == this->subphase_mod) {
            p_now->off_clutch = interpolate_float(elapsed_mod, sid->ptr_prev_pressures->off_clutch, 0, 0, 60, InterpType::Linear);
            p_now->overlap_mod = interpolate_float(elapsed_mod, sid->ptr_prev_pressures->overlap_mod, 0, 0, 60, InterpType::Linear);
            int mod_end = (
                ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc) +
                ((-centrifugal_force_off_clutch) * sid->inf.centrifugal_factor_off_clutch * sid->inf.pressure_multi_mpc) +
                (-(sid->inf.pressure_multi_mpc*150)) + // 150 comes from calibration pointer on E55 coding (TODO - Locate in calibration and assign in CAL structure)
                sid->inf.mpc_pressure_spring_reduction
            );

            p_now->mod_sol_req  = interpolate_float(elapsed_mod, sid->ptr_prev_pressures->mod_sol_req, mod_end, 0, 60, InterpType::Linear);
            if (elapsed_mod > 60) {
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

    // Do torque request calculations
    int max = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, 4000, false);
    int trq_req_raw = interpolate_float(abs_input_torque, 0, abs_input_torque/2, 0, max, InterpType::Linear);
    trq_req_raw *= interpolate_float(sd->engine_rpm, 1.0, 1.5, 1000, 4000, InterpType::Linear);

    this->set_trq_request_val(MAX(trq_req_raw, decent_adder_torque));

    // Now check if the model is active or not (Check up ramp first)

    // Calculate syncronize point
    bool sync_trigger_up = false;
    if (nullptr != this->on_clutch_delta) {
        if (ShiftHelpers::ms_till_target_on_rpm(100, this->on_clutch_delta->get_average(), sid->ptr_r_clutch_speeds->on_clutch_speed) <= 160) {
            sync_trigger_up = true;
        }
    }

    if (sync_trigger_up || sid->ptr_r_clutch_speeds->on_clutch_speed < 100) {
        this->disable_trq_request(total_elapsed);
    } else if ((phase_id == PHASE_MOMENTUM_CONTROL && subphase_shift == 0) || abs(sid->ptr_r_clutch_speeds->off_clutch_speed) > 100) {
        this->trigger_trq_request(total_elapsed);
    }

    int request_val = 0;
    request_val = this->get_trq_req_ramp_val(total_elapsed, 140, 160);

    // Disable torque request if conditions are not met (Override)
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

uint8_t CrossoverShift::max_shift_stage_id() {
    return PHASE_END_CONTROL;
}