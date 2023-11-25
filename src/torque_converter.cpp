#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcu_maths.h"
#include "nvs/eeprom_impl.h"
#include "nvs/all_keys.h"

// 1400 mBar ~= locking (C200CDI)
// 1700 mBar ~= locking (E55 AMG)

const int16_t tcc_learn_x_headers[5] = {1,2,3,4,5};
const int16_t tcc_learn_y_headers[1] = {1};
const int16_t tcc_learn_default_data[5] = {1500, 1500, 1500, 1500, 1500};

static const uint16_t TCC_ADJ_INTERVAL_MS = 500;

TorqueConverter::TorqueConverter(uint16_t max_gb_rating)  {
    tcc_learn_lockup_map = new StoredMap(NVS_KEY_TCC_LEARN_LOCK_MAP, 5, tcc_learn_x_headers, tcc_learn_y_headers, 5, 1, tcc_learn_default_data);
    if (this->tcc_learn_lockup_map->init_status() != ESP_OK) {
        delete[] this->tcc_learn_lockup_map;
    }
    int16_t* data = this->tcc_learn_lockup_map->get_current_data();
    for (int i = 0; i < 5; i++) {
        //data[i] = tcc_learn_default_data[i];
        ESP_LOGI("TCC", "Adapt value for gear %d - %d mBar", i+1, data[i]);
    }
    this->slip_average = new MovingAverage(20); // 2 Seconds moving window (Every 100ms)
    if (!this->slip_average->init_ok()) {
        delete this->slip_average;
    }
}

inline void TorqueConverter::reset_rpm_samples(SensorData* sensors) {
        //this->input_rpm_tot = this->last_input_rpm = sensors->input_rpm;
        //this->engine_rpm_tot = this->last_engine_rpm = sensors->engine_rpm;
        //this->rpm_samples = 1;
}

void TorqueConverter::set_shift_target_state(InternalTccState target_state) {
    this->shift_req_tcc_state = target_state;
}

void TorqueConverter::on_shift_ending(void) {
    this->shift_req_tcc_state = InternalTccState::None;
}

void TorqueConverter::diag_toggle_tcc_sol(bool en) {
    ESP_LOGI("TCC", "Diag request to set TCC control to %d", en);
    this->tcc_solenoid_enabled = en;
}

void TorqueConverter::adjust_map_cell(GearboxGear g, uint16_t new_pressure) {
    // Too much slip
    int16_t* modify = this->tcc_learn_lockup_map->get_current_data();
    int16_t curr_v = modify[(uint8_t)(g)-1];
    ESP_LOGI("TCC", "Adjusting TCC adaptation for gear %d. Was %d mBar, now %d mBar", (uint8_t)g, curr_v, new_pressure);
    modify[(uint8_t)(g)-1] = (int16_t)new_pressure;
    this->pending_changes = true;
}

void TorqueConverter::update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting) {
    if (!this->tcc_solenoid_enabled) {
        pm->set_target_tcc_pressure(0);
        return;
    }
    
    GearboxGear cmp_gear = curr_gear;

    // Decider 
    InternalTccState targ = InternalTccState::Open;
    bool force_lock = false;
    // See if we should be enabled in gear
    if (
        (cmp_gear == GearboxGear::First && !TCC_CURRENT_SETTINGS.enable_d1) ||
        (cmp_gear == GearboxGear::Second && !TCC_CURRENT_SETTINGS.enable_d2) ||
        (cmp_gear == GearboxGear::Third && !TCC_CURRENT_SETTINGS.enable_d3) ||
        (cmp_gear == GearboxGear::Fourth && !TCC_CURRENT_SETTINGS.enable_d4) ||
        (cmp_gear == GearboxGear::Fifth && !TCC_CURRENT_SETTINGS.enable_d5)
    ) {
        targ = InternalTccState::Open;
    } else {
        // See if we should slip or close
        if (sensors->input_rpm >= TCC_CURRENT_SETTINGS.min_locking_rpm) {
            if (sensors->pedal_pos == 0 && sensors->output_rpm >= TCC_CURRENT_SETTINGS.sailing_mode_active_rpm) {
                targ = InternalTccState::Open; // Gliding mode
            } else {
                targ = InternalTccState::Slipping;
                if (this->current_tcc_state >= InternalTccState::Slipping) {
                    // Check if we should FULLY lock
                    if (
                        (sensors->pedal_pos != 0 && ((sensors->pedal_pos*100)/250) < TCC_CURRENT_SETTINGS.locking_pedal_pos_max) || // Small pedal input (Safe to lock)
                        sensors->output_rpm > TCC_CURRENT_SETTINGS.force_lock_min_output_rpm // Force lock at very high speeds no matter the pedal position
                    ) {
                        targ = InternalTccState::Closed;
                        force_lock = sensors->output_rpm > TCC_CURRENT_SETTINGS.force_lock_min_output_rpm;
                    }
                }
            }
        } else {
            targ = InternalTccState::Open;
        }
    }
    if (is_shifting && this->shift_req_tcc_state != InternalTccState::None) {
        targ = MIN(targ, this->shift_req_tcc_state);
    }

    float calc_friction_torque = (float)sensors->static_torque / sensors->gear_ratio;
    int rpm_delta = (int)sensors->engine_rpm - (int)sensors->input_rpm;
    int idx = 0;
    switch(curr_gear) {
        case GearboxGear::Fifth:
            idx = 4;
            break;
        case GearboxGear::Fourth:
            idx = 3;
            break;
        case GearboxGear::Third:
            idx = 2;
            break;
        case GearboxGear::Second:
            idx = 1;
            break;
        default: // 1st
            idx = 0;
            break;
    }
    int output_mpc = pressure_manager->get_corrected_modulating_pressure();
    uint32_t time_since_last_adapt = GET_CLOCK_TIME() - this->last_adapt_check;
    bool at_req_pressure = this->tcc_pressure_target == this->tcc_pressure_current;
    if (targ == InternalTccState::Open || targ == InternalTccState::None) {
        this->tcc_pressure_current = 0;
        this->tcc_pressure_target = 0;
    } else if (targ == InternalTccState::Slipping) {
        if (calc_friction_torque > 0) {
            if (at_req_pressure && sensors->static_torque > 0 && sensors->static_torque < 100 && rpm_delta > 100 && time_since_last_adapt > 250) {
                if (this->slip_offset[idx] < 1000) {
                    this->slip_offset[idx] += 5;
                    this->lock_offset[idx] = MAX(this->slip_offset[idx], this->lock_offset[idx]);
                }
                this->last_adapt_check = GET_CLOCK_TIME();
            } else if (at_req_pressure && sensors->static_torque > 0 && sensors->static_torque < 100 && rpm_delta <= 30 && time_since_last_adapt > 250) {
                if (this->slip_offset[idx] > -500) {
                    this->slip_offset[idx] -= 5;
                }
                this->last_adapt_check = GET_CLOCK_TIME();
            }
            this->tcc_pressure_target = output_mpc + this->slip_offset[idx];
        }

    } else if (targ == InternalTccState::Closed) {
        if (at_req_pressure && sensors->static_torque > 0 && sensors->static_torque < 200 && rpm_delta > 20 && time_since_last_adapt > 250) {
            if (this->lock_offset[idx] < 1000) {
                this->lock_offset[idx] += 2;
            }
            this->last_adapt_check = GET_CLOCK_TIME();
        }
        this->tcc_pressure_target = output_mpc + this->lock_offset[idx];
    }
    
    if (this->tcc_pressure_target > this->tcc_pressure_current + 5 && this->current_tcc_state < targ) {
        if (targ == InternalTccState::Slipping) {
            // Just slipping, instant increase
            this->tcc_pressure_current = this->tcc_pressure_target;
        } else {
            // Increasing (Slow ramp)
            if (this->tcc_pressure_current == 0) {
                this->tcc_pressure_current = MIN((this->tcc_pressure_target/3)*2, 1000);
            } else {
                this->tcc_pressure_current += MIN(5, this->tcc_pressure_target - this->tcc_pressure_current);
            }
        }
    } else {
        // Equal or less than (Jump to requested)
        this->tcc_pressure_current = this->tcc_pressure_target;
    }
    if (this->tcc_pressure_current < 0) {
        this->tcc_pressure_current = 0;
    }
    // Change state if pressure target met
    if (this->tcc_pressure_current == this->tcc_pressure_target) {
        this->current_tcc_state = targ;
    }

    pm->set_target_tcc_pressure(this->tcc_pressure_current);
}

uint8_t TorqueConverter::progress_to_next_phase(void) {
    uint8_t ret = 100;
    return ret;
}

TccClutchStatus TorqueConverter::get_clutch_state(void) {
    TccClutchStatus ret = TccClutchStatus::Open;
    InternalTccState cmp = this->target_tcc_state;
    // Reduction or equal state (EG: Closed -> Slipping)
    // Just return the target state
    if (this->current_tcc_state >= cmp) {
        switch (this->current_tcc_state) {
            case InternalTccState::Closed:
                ret = TccClutchStatus::Closed;
                break;
            case InternalTccState::Slipping:
                ret = TccClutchStatus::Slipping;
                break;
            case InternalTccState::Open: // Already set
            default:
                break;
        }
    } 
    // Increasing state (EG: Open -> Slipping)
    else {
        if (this->current_tcc_state == InternalTccState::Open && this->target_tcc_state == InternalTccState::Slipping) {
            ret = TccClutchStatus::OpenToSlipping;
        } else if (this->current_tcc_state == InternalTccState::Slipping && this->target_tcc_state == InternalTccState::Closed) {
            ret = TccClutchStatus::SlippingToClosed;
        } else {
            // Open -> Closed (Return slipping as then TCC will be slipping which then results in slipping -> Closed)
            ret = TccClutchStatus::OpenToSlipping;
        }
    }
    return ret;
}