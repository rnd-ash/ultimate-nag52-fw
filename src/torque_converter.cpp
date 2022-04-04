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
        this->mpc_curr_compensation = 0;
        sol_tcc->write_pwm_12_bit(0);
        return;
    }

    if (this->was_idle && this->curr_tcc_pwm > 50) {
        this->was_idle = false;
        this->curr_tcc_pwm *= 0.5; // Faster response for TCC
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
            if (sensors->pedal_pos == 0) {
                // Hard coded for coasting
                max_allowed_slip = 250;
                min_allowed_slip = 50;

            }
            // This means that at 1000Rpm, we will get 0.9 the slip (More slip)
            // At 2100RPM, we will get half the slip as at 1100rpm
            // Dynamic slip :D
            //float multiplier = MIN(0.9, (float)(sensors->input_rpm-100.0)/1000.0);
            if (sensors->output_rpm > 2000) {
                max_allowed_slip /= 2;
                min_allowed_slip /= 2;
            }
        }

        if (sensors->tcc_slip_rpm > max_allowed_slip) {
            // Increase pressure
            this->curr_tcc_pwm += MAX(1, (sensors->pedal_pos/11)) * pm->get_tcc_temp_multiplier(sensors->atf_temp);
        } else if (sensors->tcc_slip_rpm < min_allowed_slip && this->curr_tcc_pwm >= 0) {
            // Decrease pressure, but only if we have pedal input
            this->curr_tcc_pwm -= 0.5 * pm->get_tcc_temp_multiplier(sensors->atf_temp);
        }
    }
    int tcc_offset = 0;
    if (this->curr_tcc_pwm != 0 ) {
        tcc_offset = 250;
    } else {
        this->mpc_curr_compensation = 0;
    }
    /*
    if (this->curr_tcc_pwm != 0 && !this->inhibit_increase) {
        uint16_t compensation_multiplier = is_shifting ? (sol_mpc->get_pwm() / 50) : 0;
        if (compensation_multiplier == 0) {
            mpc_curr_compensation = 0;
        } else if (this->mpc_curr_compensation < compensation_multiplier) {
            mpc_curr_compensation++;
        }
        this->curr_tcc_pwm += mpc_curr_compensation;
    }
    */
    if (this->curr_tcc_pwm <=- 0) { // Just to be safe!
        this->curr_tcc_pwm = 0;
    }
    sol_tcc->write_pwm_12bit_with_voltage(tcc_offset+((uint16_t)(this->curr_tcc_pwm/10)), sensors->voltage);
}

void TorqueConverter::on_shift_complete(uint64_t now) {
    this->inhibit_increase = false;
}

void TorqueConverter::on_shift_start(uint64_t now, bool is_downshift, SensorData* sensors) {
}