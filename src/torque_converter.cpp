#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcu_maths.h"
#include "nvs/eeprom_impl.h"

// 1400 mBar ~= locking (C200CDI)
// 1700 mBar ~= locking (E55 AMG)

const int16_t tcc_learn_x_headers[5] = {1,2,3,4,5};
const int16_t tcc_learn_y_headers[1] = {1};
const int16_t tcc_learn_default_data[5] = {1500, 1500, 1500, 1500, 1500};

static const uint16_t TCC_ADJ_INTERVAL_MS = 500;

TorqueConverter::TorqueConverter(uint16_t max_gb_rating)  {
    tcc_learn_lockup_map = new StoredMap("TCC_LOCK", 5, tcc_learn_x_headers, tcc_learn_y_headers, 5, 1, tcc_learn_default_data);
    if (this->tcc_learn_lockup_map->init_status() != ESP_OK) {
        delete[] this->tcc_learn_lockup_map;
    }
    int16_t* data = this->tcc_learn_lockup_map->get_current_data();
    for (int i = 0; i < 5; i++) {
        //data[i] = tcc_learn_default_data[i];
        ESP_LOGI("TCC", "Adapt value for gear %d - %d mBar", i+1, data[i]);
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

void TorqueConverter::adjust_map_cell(GearboxGear g, uint16_t new_pressure) {
    // Too much slip
    int16_t* modify = this->tcc_learn_lockup_map->get_current_data();
    int16_t curr_v = modify[(uint8_t)(g)-1];
    if (abs(curr_v-new_pressure) > 20) {
        ESP_LOGI("TCC", "Adjusting TCC adaptation for gear %d. Was %d mBar, now %d mBar", (uint8_t)g, curr_v, new_pressure);
        modify[(uint8_t)(g)-1] = (int16_t)new_pressure;
        this->pending_changes = true;
    }
}

void TorqueConverter::update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting) {
    GearboxGear cmp_gear = curr_gear;

    // Decider 
    InternalTccState targ = InternalTccState::Open;

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
                        (sensors->pedal_pos != 0 && sensors->pedal_pos < TCC_CURRENT_SETTINGS.locking_pedal_pos_max) || // Small pedal input (Safe to lock)
                        sensors->output_rpm > TCC_CURRENT_SETTINGS.force_lock_min_output_rpm // Force lock at very high speeds no matter the pedal position
                    ) {
                        targ = InternalTccState::Closed;
                    }
                }
            }
        } else {
            targ = InternalTccState::Open;
        }
    }
    if (is_shifting && this->shift_req_tcc_state != InternalTccState::None) {
        targ = MIN(this->target_tcc_state, this->shift_req_tcc_state);
    }

    this->target_tcc_state = targ;
    if (sensors->input_rpm < TCC_CURRENT_SETTINGS.min_locking_rpm) {
        // RPM too low for slipping, see if we can prefill
        if (sensors->input_rpm != 0 && sensors->engine_rpm >= TCC_CURRENT_SETTINGS.prefill_min_engine_rpm) {
            this->tcc_pressure_target = TCC_CURRENT_SETTINGS.prefill_pressure;
        } else {
            this->tcc_pressure_target = 0;
        }
        this->current_tcc_state = InternalTccState::Open;
        this->target_tcc_state = InternalTccState::Open;
    } else {
        // Safe to lock or slip
        if (this->target_tcc_state == InternalTccState::Open) {
            this->tcc_pressure_target = TCC_CURRENT_SETTINGS.prefill_pressure;
        } else if (this->target_tcc_state == InternalTccState::Slipping || this->target_tcc_state == InternalTccState::Closed) {
            this->tcc_pressure_target = this->tcc_learn_lockup_map->get_value((float)cmp_gear, 1.0); // Slip at max torque
            if (this->target_tcc_state == InternalTccState::Closed) { // Locked
                if (sensors->output_rpm > TCC_CURRENT_SETTINGS.pressure_multiplier_output_rpm.raw_min) {
                    this->tcc_pressure_target = (uint32_t)(float)this->tcc_pressure_target * scale_number(sensors->output_rpm, &TCC_CURRENT_SETTINGS.pressure_multiplier_output_rpm);
                }
                this->tcc_pressure_target *= 1.25;
            }
        }
    }

    // State reduction or same state, Immedate pressure change
    if (this->target_tcc_state < current_tcc_state || this->target_tcc_state == current_tcc_state) {
        this->tcc_pressure_current = this->tcc_pressure_target;
    } else { // State increase, ramp up pressure
        // Ramp up
        if (is_shifting) { // If shifting, don't ramp, immedietly change. The shifting of the transmission will dampen this
            this->tcc_pressure_current = this->tcc_pressure_target;
        } else {
            this->tcc_pressure_current = MIN(this->tcc_pressure_current+TCC_CURRENT_SETTINGS.pressure_increase_step, this->tcc_pressure_target);
        }
    }

    if (this->tcc_pressure_target == this->tcc_pressure_current) {
        this->current_tcc_state = this->target_tcc_state;        
    }

    if (TCC_CURRENT_SETTINGS.adapt_enable) {
        bool adapt_conditions_met = false;
        if (this->target_tcc_state  == InternalTccState::Slipping && this->current_tcc_state == InternalTccState::Slipping && !is_shifting) {
            // Try adapting when slipping is the current, and target state
            if ((sensors->static_torque > TCC_CURRENT_SETTINGS.min_torque_adapt && sensors->static_torque < TCC_CURRENT_SETTINGS.max_torque_adapt) && sensors->engine_rpm < TCC_CURRENT_SETTINGS.tcc_stall_speed) {
                adapt_conditions_met = true;
                if (sensors->current_timestamp_ms - this->last_adapt_check > TCC_CURRENT_SETTINGS.adapt_test_interval_ms) {
                    // Do the adaptation!

                    // 1. Create a min and max linear line between our max bounds
                    //    Such that the area in between min and max are the 'OK' slip region
                    int max_slip_targ = scale_number(
                        sensors->static_torque,
                        TCC_CURRENT_SETTINGS.max_slip_min_adapt_trq,
                        TCC_CURRENT_SETTINGS.max_slip_max_adapt_trq,
                        TCC_CURRENT_SETTINGS.min_torque_adapt,
                        TCC_CURRENT_SETTINGS.max_torque_adapt
                    );
                    int min_slip_targ = scale_number(
                        sensors->static_torque,
                        TCC_CURRENT_SETTINGS.min_slip_min_adapt_trq,
                        TCC_CURRENT_SETTINGS.min_slip_max_adapt_trq,
                        TCC_CURRENT_SETTINGS.min_torque_adapt,
                        TCC_CURRENT_SETTINGS.max_torque_adapt
                    );
                    float new_p = 0;
                    if (sensors->tcc_slip_rpm > max_slip_targ) {
                        // Too much clutch slip - Increase pressure
                        new_p = this->tcc_pressure_target+TCC_CURRENT_SETTINGS.adapt_pressure_step;
                    } else if (sensors->tcc_slip_rpm < min_slip_targ) {
                        // Too little clutch slip - Reduce pressure
                        new_p = this->tcc_pressure_target-TCC_CURRENT_SETTINGS.adapt_pressure_step;
                    }
                    if (new_p != 0) {
                        adjust_map_cell(curr_gear, new_p);
                        this->tcc_pressure_target = this->tcc_pressure_current = new_p;
                    }
                    this->last_adapt_check = sensors->current_timestamp_ms;
                }
                // Check if torque is within range (Should be low torque or coasting)
            }
        }
        if (!adapt_conditions_met) {
            // Set the last adapt timestamp if not adapted due to wrong conditions
            // This allows for adapting to only happen after a cooldown, when things are settled
            this->last_adapt_check = sensors->current_timestamp_ms;
        }
    }
    pm->set_target_tcc_pressure(this->tcc_pressure_current);
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