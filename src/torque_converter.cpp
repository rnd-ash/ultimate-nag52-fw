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
                        (sensors->pedal_pos != 0 && ((sensors->pedal_pos*100)/255) < TCC_CURRENT_SETTINGS.locking_pedal_pos_max) || // Small pedal input (Safe to lock)
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

    this->target_tcc_state = targ;
    if (sensors->input_rpm < TCC_CURRENT_SETTINGS.min_locking_rpm) {
        // RPM too low for slipping, see if we can prefill
        if (sensors->input_rpm > 50 && sensors->engine_rpm >= TCC_CURRENT_SETTINGS.prefill_min_engine_rpm) {
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
            if (sensors->static_torque > TCC_CURRENT_SETTINGS.max_torque_adapt) {
                this->tcc_pressure_target *= interpolate_float(sensors->static_torque, 1.0, 1.5, TCC_CURRENT_SETTINGS.max_torque_adapt, TCC_CURRENT_SETTINGS.max_torque_adapt*2, InterpType::Linear);
            }
            if (this->target_tcc_state == InternalTccState::Closed) { // Locked
                if (sensors->output_rpm > TCC_CURRENT_SETTINGS.pressure_multiplier_output_rpm.raw_min) {
                    this->tcc_pressure_target = (uint32_t)(float)this->tcc_pressure_target * interpolate_float(sensors->output_rpm, &TCC_CURRENT_SETTINGS.pressure_multiplier_output_rpm, InterpType::Linear);
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
            // If state is going to lock, we increase it A LOT slower
            uint16_t step = TCC_CURRENT_SETTINGS.pressure_increase_step;
            if (this->target_tcc_state == InternalTccState::Closed) {
                step /= 10.0;
            }
            this->tcc_pressure_current = MIN(this->tcc_pressure_current+step, this->tcc_pressure_target);
        }
    }

    if (this->tcc_pressure_target == this->tcc_pressure_current) {
        this->current_tcc_state = this->target_tcc_state;
        this->prev_state_tcc_pressure = this->tcc_pressure_current;   
    }

    if (TCC_CURRENT_SETTINGS.adapt_enable && nullptr != this->slip_average) {
        bool adapt_state = this->target_tcc_state == this->current_tcc_state && this->current_tcc_state >= InternalTccState::Slipping && !force_lock;
        bool in_adapt_torque_range = sensors->static_torque >= TCC_CURRENT_SETTINGS.min_torque_adapt && sensors->static_torque <= TCC_CURRENT_SETTINGS.max_torque_adapt;
        bool in_temp_range = sensors->atf_temp >= ADP_CURRENT_SETTINGS.min_atf_temp && sensors->atf_temp <= ADP_CURRENT_SETTINGS.max_atf_temp;
        bool loaded = sensors->static_torque >= sensors->driver_requested_torque; // Only adapt when engine is loaded
        //ESP_LOGI("TCC", "%d %d %d (%d %d)", adapt_state, in_adapt_torque_range, is_shifting, (int)this->target_tcc_state, (int)this->current_tcc_state);
        bool adapt_check = false;
        if ((!adapt_state || is_shifting || !in_adapt_torque_range) && !this->slip_average->reset_done()) {
            this->slip_average->reset();
        }
        if (adapt_state && !is_shifting && in_adapt_torque_range && in_temp_range && loaded && sensors->engine_rpm < TCC_CURRENT_SETTINGS.tcc_stall_speed) {
            adapt_check = true;
            if (GET_CLOCK_TIME() - last_slip_add_time > 100) {
                last_slip_add_time = GET_CLOCK_TIME();
                this->slip_average->add_sample((int32_t)sensors->engine_rpm - (int32_t)sensors->input_rpm);
            }
        }
        if (adapt_check && this->slip_average->has_full_samples()) {
            // Try adapting when slipping is the current, and target state
            bool should_lock = this->target_tcc_state == InternalTccState::Closed && this->current_tcc_state == InternalTccState::Closed;
            if (GET_CLOCK_TIME() - this->last_adapt_check > TCC_CURRENT_SETTINGS.adapt_test_interval_ms) {
                int slip_avg = this->slip_average->get_average();
                // int slip_now = (int32_t)sensors->engine_rpm - (int32_t)sensors->input_rpm;
                // Do the adaptation!
                // 1. Create a min and max linear line between our max bounds
                //    Such that the area in between min and max are the 'OK' slip region
                int max_slip_targ = interpolate_float(
                    sensors->static_torque,
                    TCC_CURRENT_SETTINGS.max_slip_min_adapt_trq,
                    TCC_CURRENT_SETTINGS.max_slip_max_adapt_trq,
                    TCC_CURRENT_SETTINGS.min_torque_adapt,
                    TCC_CURRENT_SETTINGS.max_torque_adapt,
                    InterpType::Linear
                );
                int min_slip_targ = interpolate_float(
                    sensors->static_torque,
                    TCC_CURRENT_SETTINGS.min_slip_min_adapt_trq,
                    TCC_CURRENT_SETTINGS.min_slip_max_adapt_trq,
                    TCC_CURRENT_SETTINGS.min_torque_adapt,
                    TCC_CURRENT_SETTINGS.max_torque_adapt,
                    InterpType::Linear
                );
                float new_p = 0;
                if (slip_avg > max_slip_targ) {
                    // Too much clutch slip - Increase pressure
                    new_p = this->tcc_pressure_target+TCC_CURRENT_SETTINGS.adapt_pressure_step;
                } else if (slip_avg < min_slip_targ && !should_lock) {
                    // Too little clutch slip - Reduce pressure
                    // DO NOT ADJUST BELOW PREFILL PRESSURE.
                    if (this->tcc_pressure_target-TCC_CURRENT_SETTINGS.adapt_pressure_step > TCC_CURRENT_SETTINGS.prefill_pressure) {
                        new_p = this->tcc_pressure_target-TCC_CURRENT_SETTINGS.adapt_pressure_step;
                    }
                }
                if (new_p != 0) {
                    adjust_map_cell(curr_gear, new_p);
                    this->tcc_pressure_target = this->tcc_pressure_current = new_p;
                }
                this->last_adapt_check = GET_CLOCK_TIME();
            }
        }
        if (!adapt_check) {
            // Set the last adapt timestamp if not adapted due to wrong conditions
            // This allows for adapting to only happen after a cooldown, when things are settled
            this->last_adapt_check = GET_CLOCK_TIME();
        }
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