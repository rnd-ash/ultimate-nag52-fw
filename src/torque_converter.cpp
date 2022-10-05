#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcm_maths.h"
#include "macros.h"

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

// 1400 Mbar ~= locking

#define TCC_PREFILL 500 // mBar

void TorqueConverter::update(GearboxGear curr_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting) {
    if (sensors->input_rpm < 1000) { // RPM too low!
        if (sensors->engine_rpm > 900) {
            if (!this->prefilling) {
                this->prefilling = true;
                prefill_start_time = sensors->current_timestamp_ms;
                this->curr_tcc_pressure = 0;
                pm->set_target_tcc_pressure(TCC_PREFILL); // Being early prefil
            }
            this->state = ClutchStatus::Open;
            strike_count = 0;
            return;
        }
        this->was_idle = true;
        pm->set_target_tcc_pressure(0);
        prefilling = false;
        this->state = ClutchStatus::Open;
        strike_count = 0;
        return;
    }
    if (this->was_idle) {
        if (!prefilling) {
            this->prefilling = true;
            prefill_start_time = sensors->current_timestamp_ms;
        }
        this->was_idle = false;
        this->curr_tcc_pressure = TCC_PREFILL;
        this->state = ClutchStatus::Open;
        strike_count = 0;
    }
    if (sensors->current_timestamp_ms - prefill_start_time < 1000) {
        pm->set_target_tcc_pressure(TCC_PREFILL);
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
    float temp_time_multiplier = pm->get_tcc_temp_multiplier(sensors->atf_temp);
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
    float rpm_multi = 2000.0/sensors->engine_rpm;
    if (is_shifting || sensors->current_timestamp_ms-sensors->last_shift_time < 1500) {
        this->last_inc_time = sensors->current_timestamp_ms;
        strike_count = 0;
        goto write_pressure;
    }
    if (slip > max_allowed_slip && strike_count < 20) {
        strike_count++;
    } else if (slip < max_allowed_slip) {
        strike_count = 0;
    }
    // Quick activate of slip mode
    if (sensors->input_rpm > 1000 && this->curr_tcc_pressure < 1100) {
        this->curr_tcc_pressure = 1100;
        goto write_pressure;
    }
    if (sensors->current_timestamp_ms - last_inc_time > 500 * rpm_multi * temp_time_multiplier && strike_count >= 10) {
        if (slip > max_allowed_slip) {
            if (this->curr_tcc_pressure < 1100) {
                this->curr_tcc_pressure += 50; // 50mbar
            } else if (this->curr_tcc_pressure >= 1200) {
                this->curr_tcc_pressure += 10; // 50mbar
            } else {
                this->curr_tcc_pressure += 25; // 50mbar
            }
            this->last_inc_time = sensors->current_timestamp_ms;
        } else if (sensors->tcc_slip_rpm < min_allowed_slip && this->curr_tcc_pressure >= 1100 && sensors->static_torque > 40) {
            // Decrease pressure, but only if we have pedal input
            this->curr_tcc_pressure -= 50; // 50mBar
            this->last_inc_time = sensors->current_timestamp_ms;
        }
    }
write_pressure:
    if (this->curr_tcc_pressure <= 0) { // Just to be safe!
        this->curr_tcc_pressure = 0;
    }
    this->state = (slip > 100 || is_shifting) ? ClutchStatus::Slipping : ClutchStatus::Closed;
    pm->set_target_tcc_pressure((uint16_t)(this->curr_tcc_pressure+this->tcc_shift_adder));
}

void TorqueConverter::on_shift_complete(uint64_t now) {
    this->tcc_shift_adder = 0;
}

// 1600 - lock
// 1000 - slip
// 500 - prefill
void TorqueConverter::on_shift_start(uint64_t now, bool is_downshift, SensorData* sensors, float shift_firmness) {
    if (this->curr_tcc_pressure < 1200) {
        this->curr_tcc_pressure = 1200;
    } else if (sensors->static_torque > 0 && abs(sensors->tcc_slip_rpm) < 20) {
        uint32_t additional_reduction = scale_number(shift_firmness*10, 100, 0, 0, 100);
        this->curr_tcc_pressure -= additional_reduction;
    }
}

ClutchStatus TorqueConverter::get_clutch_state() {
    return this->state;
}