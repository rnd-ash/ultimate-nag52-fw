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

void TorqueConverter::update(GearboxGear curr_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting) {
    if (sensors->input_rpm < 1000) { // RPM too low!
        this->was_idle = true;
        sol_tcc->write_pwm_12_bit(0);
        return;
    }

    if (this->was_idle && this->curr_tcc_pwm > 50) {
        this->was_idle = false;
        this->curr_tcc_pwm -= 50; // Faster response for TCC
    }

    if (!is_shifting) {
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
        }

        if (sensors->tcc_slip_rpm > max_allowed_slip) {
            // Increase pressure
            this->curr_tcc_pwm += MAX(1, (sensors->pedal_pos/11));
        } else if (sensors->tcc_slip_rpm < min_allowed_slip && this->curr_tcc_pwm >= 2 && sensors->pedal_pos > 10) {
            // Decrease pressure, but only if we have pedal input
            this->curr_tcc_pwm -= 1;
        }
    }
    int tcc_offset = 0;
    if (this->curr_tcc_pwm != 0 ) {
        tcc_offset = 300;
    }
    if (is_shifting && tcc_offset != 0 && !this->inhibit_increase) {
        tcc_offset += sol_mpc->get_pwm() / 20;
    }
    sol_tcc->write_pwm_12bit_with_voltage(tcc_offset+(this->curr_tcc_pwm/10), sensors->voltage);
}

void TorqueConverter::on_shift_complete(uint64_t now) {
    this->inhibit_increase = false;
}

void TorqueConverter::on_shift_start(uint64_t now, bool is_downshift, float shift_firmness, SensorData* sensors) {
    if (is_downshift && sensors->pedal_pos == 0) {
        this->inhibit_increase = true;
    }
}