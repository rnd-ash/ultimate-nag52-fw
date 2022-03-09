#include "torque_converter.h"
#include "solenoids/solenoids.h"

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

void TorqueConverter::update(GearboxGear curr_gear, PressureManager* pm, LockupType max_lockup, SensorData* sensors, bool is_shifting) { // Only called when NOT changing gears
    uint8_t gear_id = get_gear_idx(curr_gear);
    uint8_t atf_id = get_temp_idx(sensors->atf_temp);
    uint64_t now = sensors->current_timestamp_ms;
    if (gear_idx == 0xFF) { return; } // Invalid gear for TCC code
    if (sensors->input_rpm < 1000) { // RPM too low!
        this->curr_tcc_pwm = 0;
        this->targ_tcc_pwm = 0; // Armed
        this->current_lockup = LockupType::Open;
        this->was_idle = true;
        sol_tcc->write_pwm_12_bit(0);
        return;
    }
    if (max_lockup == LockupType::Open) { // Do nothing
        this->curr_tcc_pwm = 0;
        this->targ_tcc_pwm = 0;
        this->current_lockup = LockupType::Open;
        sol_tcc->write_pwm_12_bit(0);
        return; 
    }
    // What kind of lockup do we want?
    this->current_lockup = LockupType::Slip;
    this->targ_tcc_pwm = (torque_converter_adaptation[this->gear_idx].slip_values[atf_id]*10) ; // For 40(ish) Nm

    // Different gear change?
    if (this->gear_idx != gear_id) {
        //ESP_LOGI("TCC", "GEAR_CHG");
        this->gear_idx = gear_id;
    }
    if (was_idle) {
        this->curr_tcc_pwm = 1000;
        this->was_idle = false;
    } else if (this->curr_tcc_pwm < this->targ_tcc_pwm) { // Smooth low->High TCC PWM (Under gas only)
        if (sensors->pedal_pos != 0 && sensors->tcc_slip_rpm > 50) {
            this->curr_tcc_pwm += sensors->pedal_pos/10 * pm->get_tcc_temp_multiplier(sensors->atf_temp);
        }
    } else if (this->curr_tcc_pwm > this->targ_tcc_pwm) { // Jump High->Low TCC PWM
        this->curr_tcc_pwm-=10;
    } else if (this->curr_tcc_pwm == this->targ_tcc_pwm) { // Adapt the current value (Only when in stable gear)
        // Same gear, check adaptation
        if (now-this->last_modify_time >= 500) {
            if (!torque_converter_adaptation[gear_id].learned[atf_id]) {
                // Check how our slip is doing (Must slip when engine is idle and moving)
                if ((sensors->static_torque >= 100 && sensors->static_torque <= 180 && sensors->input_rpm < 2500 && sensors->pedal_pos < 125)) {
                    last_modify_time = now;
                    adapting_load = true;
                    adapting_idle = false;
                    idle_adapt_time = 0;
                    load_adapt_time += 500;
                    if (abs(sensors->tcc_slip_rpm) > 200) {
                        ESP_LOGI("TCC","ADAPT-SLIP");
                        this->curr_tcc_pwm+=20; // 4 = 1/256 PWM
                        this->targ_tcc_pwm+=20;
                        this->pending_changes = true; // Say the map has been modified (Needs to commit to NVS)
                        // Change all the ones higher up
                        for (int gid = gear_id; gid < NUM_GEARS; gid++) {
                            for (int idx = atf_id; idx < 17; idx++) {
                                if (torque_converter_adaptation[gid].slip_values[idx] < this->curr_tcc_pwm/10 && !torque_converter_adaptation[gid].learned[idx]) {
                                    torque_converter_adaptation[gid].slip_values[idx] = this->curr_tcc_pwm/10;
                                }
                            }
                        }
                    }
                } else if (sensors->static_torque < 0 && sensors->input_rpm > 1100 && sensors->pedal_pos == 0) {
                    last_modify_time = now;
                    adapting_load = false;
                    adapting_idle = true;
                    idle_adapt_time += 500;
                    load_adapt_time = 0;
                    if (abs(sensors->tcc_slip_rpm) > 50) {
                        ESP_LOGI("TCC","ADAPT-IDLE");
                        this->curr_tcc_pwm+=10; // smaller changes when idle comparing
                        this->targ_tcc_pwm+=10;
                        this->pending_changes = true; // Say the map has been modified (Needs to commit to NVS)
                        for (int gid = gear_id; gid < NUM_GEARS; gid++) {
                            for (int idx = atf_id; idx < 17; idx++) {
                                if (torque_converter_adaptation[gid].slip_values[idx] < this->curr_tcc_pwm/10 && !torque_converter_adaptation[gid].learned[idx]) {
                                    torque_converter_adaptation[gid].slip_values[idx] = this->curr_tcc_pwm/10;
                                }
                            }
                        }
                    } else if (adapting_idle && idle_adapt_time > 3000) { // 3 second confirm!
                        ESP_LOGI("TCC", "ADAPTATION COMPLETE! Gear %d ATF_ID %d", gear_id+1, atf_id);
                        torque_converter_adaptation[gear_id].learned[atf_id] = true;
                    }
                } else {
                    adapting_idle = false;
                    adapting_load = false;
                    load_adapt_time = 0;
                    idle_adapt_time = 0;
                    last_modify_time = now;
                }
            } else { // Now we have learned, we can do more ;)

            }
        }
    }
    int tcc_offset = 0;
    if (this->curr_tcc_pwm != 0 ) {
        tcc_offset = 300;
    }
    if (!is_shifting) {
        sol_tcc->write_pwm_12bit_with_voltage(tcc_offset+(this->curr_tcc_pwm/10) + this->last_mpc_pwm/4, sensors->voltage);
    }
}

void TorqueConverter::save_adaptation_data() {
    //if (this->pending_changes) { // Save writing to NVS <3
        EEPROM::save_nvs_tcc_adaptation(torque_converter_adaptation, sizeof(torque_converter_adaptation));
    //}
}

void TorqueConverter::on_shift_complete(uint64_t now) {
    this->last_modify_time = 0;
}

void TorqueConverter::on_shift_start(uint64_t now, bool is_downshift, float shift_firmness, SensorData* sensors) {
    if (this->curr_tcc_pwm != 0 && !is_downshift) {
        this->curr_tcc_pwm *= 1.2;
        sol_tcc->write_pwm_12bit_with_voltage(300+(this->curr_tcc_pwm/10), sensors->voltage);
    }
    /*
    if (is_downshift) {
        if (sensors->tcc_slip_rpm >= 0 && sensors->tcc_slip_rpm < 150) { // Only reduce if not slipping
            if (this->curr_tcc_pwm > 2500 && this->targ_tcc_pwm > 2500) {
                this->targ_tcc_pwm -= 2000;
                this->curr_tcc_pwm -= 2000;
            }
        }
    } else {
        if (this->curr_tcc_pwm > 3500) {
            this->curr_tcc_pwm = 3500;
        }
    }
    this->last_modify_time = 0;
    */
}