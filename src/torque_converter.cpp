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

#define TCC_PREFILL 350

void TorqueConverter::update(GearboxGear curr_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting, int mpc_offset) {
    if (sensors->input_rpm < 1000) { // RPM too low!
        this->was_idle = true;
        this->mpc_curr_compensation = 0;
        pm->set_target_tcc_percent(0, -1);
        //sol_tcc->write_pwm_12_bit(0);
        prefilling = false;
        return;
    }

    uint64_t now = esp_timer_get_time() / 1000;
    if (this->was_idle && !prefilling) {
        this->was_idle = false;
        this->prefilling = true;
        prefill_start_time = sensors->current_timestamp_ms;
        this->curr_tcc_pwm = 0;
    }
    if (sensors->current_timestamp_ms - prefill_start_time < 3000) {
        pm->set_target_tcc_percent(TCC_PREFILL/4, -1);
        //sol_tcc->write_pwm_12bit_with_voltage(TCC_PREFILL, sensors->voltage);
        return;
    }

    int trq = sensors->static_torque;
    if (trq < 0) {
        trq *= -1;
    }
    // Only think about lockup on positive torque
    int max_allowed_slip;
    int min_allowed_slip;
    if (profile == nullptr) {
        max_allowed_slip = MAX(100, trq);
        min_allowed_slip = MAX(10, trq/2);
    } else {
        TccLockupBounds bounds = profile->get_tcc_lockup_bounds(sensors, curr_gear);
        max_allowed_slip = bounds.max_slip_rpm;
        min_allowed_slip = bounds.min_slip_rpm;
        // This means that at 1000Rpm, we will get 0.9 the slip (More slip)
        // At 2100RPM, we will get half the slip as at 1100rpm
        // Dynamic slip :D
        float multiplier = MIN(0.9, (float)(sensors->input_rpm/1000.0));
        max_allowed_slip *= multiplier;
        min_allowed_slip *= multiplier;
    }
    int slip = sensors->tcc_slip_rpm;
    if (is_shifting || mpc_offset != 0 || sensors->current_timestamp_ms-sensors->last_shift_time < 1500) {
        goto write_pwm;
    }
    if (slip > max_allowed_slip) {
        int midpoint = (max_allowed_slip+min_allowed_slip*2)/3;
        float diff = slip - midpoint;
        // Increase pressure
        this->curr_tcc_pwm += MAX(0.1, (diff/15.0)) * pm->get_tcc_temp_multiplier(sensors->atf_temp);
    } else if (sensors->tcc_slip_rpm < min_allowed_slip && this->curr_tcc_pwm >= 0 && sensors->static_torque > 80) {
        // Decrease pressure, but only if we have pedal input
        this->curr_tcc_pwm -= 0.5 * pm->get_tcc_temp_multiplier(sensors->atf_temp);
    }
write_pwm:
    if (this->curr_tcc_pwm <=- 0) { // Just to be safe!
        this->curr_tcc_pwm = 0;
    }
    pm->set_target_tcc_percent((TCC_PREFILL+((uint16_t)(this->curr_tcc_pwm/10)))/4, -1);
    //sol_tcc->write_pwm_12bit_with_voltage(TCC_PREFILL+((uint16_t)(this->curr_tcc_pwm/10)), sensors->voltage);
}

void TorqueConverter::on_shift_complete(uint64_t now) {
    this->inhibit_increase = false;
}

void TorqueConverter::on_shift_start(uint64_t now, bool is_downshift, SensorData* sensors) {
    if (is_downshift) {
        this->inhibit_increase = true;
    }
}