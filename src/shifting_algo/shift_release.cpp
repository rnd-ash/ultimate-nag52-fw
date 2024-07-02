#include "shift_release.h"

const uint8_t PHASE_BLEED     = 0;
// Fill and release (FAR) (Same time!)
const uint8_t PHASE_FAR_LOWER_PRESSURE = 1;
const uint8_t PHASE_FAR_RELEASE = 2;
const uint8_t PHASE_FAR_RAMP_INTERCEPT = 3;
const uint8_t PHASE_FAR_INTERCEPT = 4;
// Securing phases
const uint8_t PHASE_OVERLAP = 5;
const uint8_t PHASE_MAX_PRESSURE  = 6;
const uint8_t PHASE_END_CONTROL   = 7;

const uint8_t FILL_RAMP_TIME = 60;
const uint8_t FILL_HOLD_TIME = 100;

ReleasingShift::ReleasingShift(ShiftInterfaceData* data) : ShiftingAlgorithm(data) {
    // 5 so we have a 5 cycle (50ms) moving avg
    this->trq_req_buffer = new FirstOrderAverage<uint16_t>(5);
}
ReleasingShift::~ReleasingShift() {
    delete this->trq_req_buffer;
}

uint8_t ReleasingShift::step(
    uint8_t phase_id,
    uint16_t abs_input_torque,
    int16_t static_torque_no_reduction,
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
    int max_trq_on_clutch  = pm->calc_max_torque_for_clutch(sid->targ_g, sid->applying, sid->ptr_w_pressures->on_clutch);
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
            sid->tcc->set_shift_target_state(InternalTccState::Open);
            ret = PHASE_FAR_LOWER_PRESSURE;
        }
    } else if (phase_id == PHASE_FAR_LOWER_PRESSURE) {
        int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, abs_input_torque);
        // Clutches
        p_now->on_clutch = sid->prefill_info.fill_pressure_on_clutch;
        p_now->off_clutch = wp_old_clutch;
        // Valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);

        if (phase_elapsed > sid->prefill_info.fill_time) {
            ret = PHASE_FAR_RELEASE;
        }
    } else if (phase_id == PHASE_FAR_RELEASE) {
#define RAMP_TIME 100
        int wp_old_clutch = pm->find_releasing_pressure_for_clutch(sid->curr_g, sid->releasing, abs_input_torque);
        // New clutch goes to low filling pressure
        // Old clutch gets reduced to as low as it can go
        p_now->on_clutch = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->on_clutch, sid->prefill_info.low_fill_pressure_on_clutch, 0, RAMP_TIME, InterpType::Linear);
        p_now->off_clutch = wp_old_clutch;
        // Valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
        if (phase_elapsed > RAMP_TIME || sid->ptr_r_clutch_speeds->on_clutch_speed > 100) {
            ret = PHASE_FAR_RAMP_INTERCEPT;
        }
    } else if (phase_id == PHASE_FAR_RAMP_INTERCEPT) {
        p_now->on_clutch = sid->prefill_info.low_fill_pressure_on_clutch;
        // Reduce to 0 (Disable the old clutch)
        p_now->off_clutch = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->off_clutch, 0, 0, RAMP_TIME, InterpType::Linear);
        // Valves
        p_now->overlap_mod = p_now->off_clutch + sid->spring_off_clutch;
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
       if (phase_elapsed > RAMP_TIME) {
            ret = PHASE_FAR_INTERCEPT;
        }
    } else if (phase_id == PHASE_FAR_INTERCEPT) {
        int duration = MIN(250, sid->chars.target_shift_time/2);
        int min_new_clutch = pm->find_releasing_pressure_for_clutch(sid->targ_g, sid->applying, abs_input_torque);

        p_now->on_clutch = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->on_clutch, min_new_clutch, 0, duration, InterpType::Linear);
        p_now->off_clutch = 0;
        // Valves
        p_now->overlap_mod = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->overlap_mod, 0, 0, duration, InterpType::Linear);
        p_now->overlap_shift = p_now->on_clutch + sid->spring_on_clutch;
        // Solenoids
        p_now->shift_sol_req = p_now->overlap_shift;
        p_now->mod_sol_req = MAX(
            ((p_now->overlap_shift - centrifugal_force_on_clutch) * sid->inf.pressure_multi_spc)+
            ((p_now->overlap_mod - centrifugal_force_off_clutch) * sid->inf.pressure_multi_mpc * sid->inf.centrifugal_factor_off_clutch)+
            sid->inf.mpc_pressure_spring_reduction
        , 0);
        if (phase_elapsed > duration || sid->ptr_r_clutch_speeds->on_clutch_speed < sid->ptr_r_clutch_speeds->off_clutch_speed) {
            ret = PHASE_OVERLAP;
        }
    } else if (phase_id == PHASE_OVERLAP) {
        int duration = MIN(250, sid->chars.target_shift_time/2);
        // New clutch gets full pressure (+momentum)
        // Old clutch valve is empties completely
        int targ_t = abs_input_torque + torque_adder + pm->find_decent_adder_torque(sid->change, abs(static_torque_no_reduction), sd->output_rpm);
        int wp_new_clutch = pm->find_releasing_pressure_for_clutch(sid->targ_g, sid->applying, targ_t);

        p_now->on_clutch = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->on_clutch, wp_new_clutch, 0, duration, InterpType::Linear);
        p_now->off_clutch = 0;

        p_now->overlap_mod = 0;
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
        p_now->overlap_mod = interpolate_float(phase_elapsed, sid->ptr_prev_pressures->mod_sol_req, wp_gear, 0, 250, InterpType::Linear);
        if (phase_elapsed > 250) {
            ret = STEP_RES_END_SHIFT;
        }
    } else {
        ret = STEP_RES_END_SHIFT; // WTF? Should never happen
    }

     // Limiting engine torque by this much (Computed later with indicated_torque - trq_req_targ = trq request output)
    int trq_req_targ = 0;
    // We can only request if engine torque is above engines min torque
    if (static_torque_no_reduction > sd->min_torque && static_torque_no_reduction > sd->min_torque && sd->input_rpm > 1200) {
        bool inc = phase_id == PHASE_MAX_PRESSURE;
        if (phase_id == PHASE_FAR_LOWER_PRESSURE) {
            if (static_torque_no_reduction > max_trq_off_clutch) {
                trq_req_targ = static_torque_no_reduction - max_trq_off_clutch;
            }
        } else if (phase_id > PHASE_FAR_LOWER_PRESSURE && phase_id < PHASE_END_CONTROL) {
            int ramp_trq = pm->find_freeing_torque(sid->change, static_torque_no_reduction, sd->output_rpm);
            if (static_torque_no_reduction > max_trq_on_clutch) {
                // Limiting due to torque restriction
                int limit = static_torque_no_reduction - max_trq_on_clutch;
                trq_req_targ = MAX(limit, ramp_trq);
            } else {
                // Limiting for ramp
                trq_req_targ = ramp_trq;
            }
        }
        this->trq_req_buffer->add_sample(trq_req_targ);
        uint16_t v = this->trq_req_buffer->get_average();
        if (v <= 1) {
            // No request
            sid->ptr_w_trq_req->amount = 0;
            sid->ptr_w_trq_req->bounds = TorqueRequestBounds::LessThan;
            sid->ptr_w_trq_req->ty = TorqueRequestControlType::None;
        } else {
            v = MIN(v, sd->driver_requested_torque*0.8);
            sid->ptr_w_trq_req->amount = sd->driver_requested_torque - v;
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