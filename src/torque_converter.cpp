#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcm_maths.h"

int get_gear_idx(GearboxGear g) {
    switch (g) {
        case GearboxGear::First:
            return 0;
        case GearboxGear::Second:
            return 1;
        case GearboxGear::Third:
            return 2;
        case GearboxGear::Fourth:
            return 3;
        case GearboxGear::Fifth:
            return 4;
        default:
            return 0xFF; // Invalid
    }
}

int get_temp_idx(int temp_raw) {
    if (temp_raw < 0) {
        return 0;
    } else if (temp_raw > 160) {
        return 16;
    }
    return temp_raw/10;
}

#define TCC_PREFILL 360

void TorqueConverter::update(GearboxGear curr_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting, int mpc_offset) {
    if (sensors->input_rpm < 1000) { // RPM too low!
        if (sensors->engine_rpm > 900) {
            if (!this->prefilling) {
                this->prefilling = true;
                prefill_start_time = sensors->current_timestamp_ms;
                this->curr_tcc_pwm = 0;
                pm->set_target_tcc_pwm(TCC_PREFILL); // Being early prefil
            }
            this->state = ClutchStatus::Open;
            return;
        }
        this->was_idle = true;
        this->mpc_curr_compensation = 0;
        pm->set_target_tcc_pwm(0);
        prefilling = false;
        this->state = ClutchStatus::Open;
        return;
    }
    if (this->was_idle) {
        if (!prefilling) {
            this->prefilling = true;
            prefill_start_time = sensors->current_timestamp_ms;
        }
        this->was_idle = false;
        this->curr_tcc_pwm = 0;
        this->state = ClutchStatus::Open;
    }
    if (sensors->current_timestamp_ms - prefill_start_time < 1000) {
        pm->set_target_tcc_pwm(TCC_PREFILL);
        this->state = ClutchStatus::Open;
        return;
    }

    int trq = sensors->static_torque;
    if (trq < 0) {
        trq *= -1;
    }
    // Only think about lockup on positive torque
    int max_allowed_slip;
    int min_allowed_slip;
    float temp_force_multiplier = pm->get_tcc_temp_multiplier(sensors->atf_temp);
    float temp_time_multiplier = 1.0/temp_force_multiplier;
    if (profile == nullptr) {
        max_allowed_slip = MAX(100, trq);
        min_allowed_slip = MAX(10, trq/2);
    } else {
        TccLockupBounds bounds = profile->get_tcc_lockup_bounds(sensors, curr_gear);
        max_allowed_slip = bounds.max_slip_rpm;
        min_allowed_slip = bounds.min_slip_rpm;
        // When producing less thats 44Nm, we should always be locked
        // as that is the turbines inertia force, if we are not then we trigger
        // more pressure with this code 
        if (sensors->static_torque <= 44 && sensors->static_torque > 0) {
            max_allowed_slip = 10;
            min_allowed_slip = 0;
        }
        // This means that at 1000Rpm, we will get 0.9 the slip (More slip)
        // At 2100RPM, we will get half the slip as at 1100rpm
        // Dynamic slip :D
        float multiplier = MIN(0.9, (float)(1000.0/MAX(1000.0, sensors->engine_rpm)));
        max_allowed_slip *= multiplier;
        min_allowed_slip *= multiplier;
    }
    int slip = abs(sensors->tcc_slip_rpm);
    if (is_shifting) { //|| sensors->current_timestamp_ms-sensors->last_shift_time < 3500*temp_time_multiplier) {
        this->last_inc_time = sensors->current_timestamp_ms;
        goto write_pwm;
    }
    if (sensors->current_timestamp_ms - last_inc_time > 500*temp_time_multiplier) {
        if (slip > max_allowed_slip) {
            int midpoint = (max_allowed_slip+min_allowed_slip*2)/3;
            float diff = slip - midpoint;
            // Increase pressure
            // Take into account delta of slip
            // if delta large, then engine RPM is jumping but clutch is slipping way more
            // if delta is small, then we only need a tiny adjustment as it is slipping but gripping
            this->curr_tcc_pwm += MAX(0.1, (diff/4.0)) * temp_force_multiplier;
            this->last_inc_time = sensors->current_timestamp_ms;
        } else if (sensors->tcc_slip_rpm < min_allowed_slip && this->curr_tcc_pwm >= 0 && sensors->static_torque > 40) {
            // Decrease pressure, but only if we have pedal input
            this->curr_tcc_pwm -= 2 * temp_force_multiplier;
            this->last_inc_time = sensors->current_timestamp_ms;
        }
    }
write_pwm:
    if (this->curr_tcc_pwm <=- 0) { // Just to be safe!
        this->curr_tcc_pwm = 0;
    }
    this->state = (slip > 100 || is_shifting) ? ClutchStatus::Slipping : ClutchStatus::Closed;
    pm->set_target_tcc_pwm(TCC_PREFILL+(uint16_t)(this->curr_tcc_pwm/10.0));
}

void TorqueConverter::on_shift_complete(uint64_t now) {
    this->inhibit_increase = false;
}

void TorqueConverter::on_shift_start(uint64_t now, bool is_downshift, SensorData* sensors) {
    // TODO any extra code for when ratio starts to change
    if (sensors->tcc_slip_rpm < 50 && sensors->static_torque > 44) {
        this->curr_tcc_pwm *= 0.95;
    }
}

ClutchStatus TorqueConverter::get_clutch_state() {
    return this->state;
}