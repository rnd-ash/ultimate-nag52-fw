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
    if (temp_raw < 40) {
        return 0;
    } else if (temp_raw < 80) {
        return 1;
    } else if (temp_raw < 120) {
        return 2;
    } else {
        return 3;
    }
}

void TorqueConverter::update(GearboxGear curr_gear, LockupType max_lockup,SensorData* sensors, bool shifting) {
    uint8_t gear_id = get_gear_idx(curr_gear);
    uint8_t atf_id = get_temp_idx(sensors->atf_temp);
    if (gear_idx == 0xFF) { return; } // Invalid gear for TCC code
    if (sensors->input_rpm < 1000) { // RPM too low!
        this->curr_tcc_percent = 0;
        this->targ_tcc_percent = 0; // Armed
        this->current_lockup = LockupType::Open;
        this->was_idle = true;
        sol_tcc->write_pwm(0);
        return;
    }
    if (max_lockup == LockupType::Open) { // Do nothing
        this->curr_tcc_percent = 0;
        this->targ_tcc_percent = 0;
        this->current_lockup = LockupType::Open;
        sol_tcc->write_pwm(0);
        return; 
    }
    // What kind of lockup do we want?
    this->current_lockup = LockupType::Slip;
    this->targ_tcc_percent = torque_converter_adaptation[this->gear_idx].slip_values[atf_id];
    
    // Different gear change?
    if (this->gear_idx != gear_id) {
        ESP_LOGI("TCC", "GEAR_CHG");
        this->gear_idx = gear_id;
    }
    if (was_idle) {
        ESP_LOGI("TCC", "WAS_IDLE");
        this->curr_tcc_percent = torque_converter_adaptation[this->gear_idx].slip_values[atf_id] * 0.9;
        this->was_idle = false;
    } else if (this->curr_tcc_percent < this->targ_tcc_percent && sensors->pedal_pos != 0) { // Smooth low->High TCC PWM (Under gas only)
        // Transitioning, trying to avoid slamming
        unsigned long now = esp_timer_get_time()/1000;
        if (shifting) {
            this->curr_tcc_percent = this->targ_tcc_percent;
            last_modify_time = now;
        }
        int timeframe = (250*3-(sensors->pedal_pos*3))+10;
        if (timeframe > 400) {
            timeframe = 400;
        }
        if (now - last_modify_time > timeframe) {
            ESP_LOGI("TCC","SMOOTHING %d to %d", curr_tcc_percent, targ_tcc_percent);
            last_modify_time = now;
            if (this->targ_tcc_percent-this->curr_tcc_percent < 3) {
                this->curr_tcc_percent++;
            } else {
                this->curr_tcc_percent += 3;
            }
        }
    } else if (this->curr_tcc_percent > this->targ_tcc_percent) { // Jump High->Low TCC PWM
        ESP_LOGI("TCC","JUMP %d to %d", curr_tcc_percent, targ_tcc_percent);
        this->curr_tcc_percent = this->targ_tcc_percent;
    } else if (!shifting && this->curr_tcc_percent == this->targ_tcc_percent) { // Adapt the current value (Only when in stable gear)
        TccAdaptationData* curr_adaptation = &torque_converter_adaptation[gear_idx];
        unsigned long now = esp_timer_get_time()/1000;
        // Same gear, check adaptation
        //ESP_LOGI("CHECK", "%d ms %d RPM, %d Nm", (int)(now-this->last_modify_time), abs(sensors->tcc_slip_rpm), sensors->static_torque);
        if (now-this->last_modify_time >= 750) {
            // Check how our slip is doing (Must slip when engine is idle and moving)
            if ((sensors->pedal_pos == 0 && abs(sensors->tcc_slip_rpm) > 150)) {
                ESP_LOGI("TCC","ADAPT-SLIP");
                this->curr_tcc_percent+=3;
                this->targ_tcc_percent+=3;
                last_modify_time = now;
                // Change all the ones higher up
                for (int idx = atf_id; idx < 17; idx++) {
                    curr_adaptation->slip_values[idx] = this->curr_tcc_percent;
                    if (curr_adaptation->lockup_values[idx] < this->curr_tcc_percent) {
                        curr_adaptation->lockup_values[idx] = this->curr_tcc_percent;
                    }
                }
            }
        }
    }
    sol_tcc->write_pwm_percent_with_voltage(this->curr_tcc_percent, sensors->voltage);
}

void TorqueConverter::save_adaptation_data() {
    EEPROM::save_nvs_tcc_adaptation(torque_converter_adaptation, sizeof(torque_converter_adaptation));
}