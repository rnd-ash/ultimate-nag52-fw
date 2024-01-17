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

const uint16_t MAX_WORKING_PRESSURE = 15000; // mBar - Same in all 722.6 variants.
const uint16_t SLIPPING_SLIP_MAX_VALUE = 100; // RPM
const uint16_t CLOSED_SLIP_MAX_VALUE = 10; // RPM


TorqueConverter::TorqueConverter(uint16_t max_gb_rating)  {
    this->rated_max_torque = max_gb_rating;

    this->slip_2_3 = new LookupTable(this->load_header, 7, slip_data_default, 7);
    this->slip_4_5 = new LookupTable(this->load_header, 7, slip_data_default, 7);
    this->lock_2_3 = new LookupTable(this->load_header, 7, lock_data_default, 7);
    this->lock_4_5 = new LookupTable(this->load_header, 7, lock_data_default, 7);

    init_tables_ok = this->slip_2_3 != nullptr && this->slip_4_5 != nullptr && this->lock_2_3 != nullptr && this->lock_4_5 != nullptr;

    if (!init_tables_ok) {
        ESP_LOGE("TCC", "Adaptation table(s) for TCC failed to load. TCC will be non functional");
    }

    this->slip_average = new MovingAverage(20); // 2 Seconds moving window (Every 100ms)
    if (!this->slip_average->init_ok()) {
        delete this->slip_average;
    }
}

void TorqueConverter::set_shift_target_state(InternalTccState target_state) {
    this->shift_req_tcc_state = target_state;
    this->is_shifting = true;
}

void TorqueConverter::on_shift_ending(void) {
    this->is_shifting = false;
}

void TorqueConverter::diag_toggle_tcc_sol(bool en) {
    ESP_LOGI("TCC", "Diag request to set TCC control to %d", en);
    this->tcc_solenoid_enabled = en;
}

void TorqueConverter::update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors) {
    // TCC is commanded to be off,
    // or adaptation table failure.
    if (!this->tcc_solenoid_enabled || !init_tables_ok) {
        pm->set_target_tcc_pressure(0);
        return;
    }
    
    GearboxGear cmp_gear = curr_gear;
    if (GET_CLOCK_TIME() - this->last_slip_add_time > 40) {
        this->slip_average->add_sample((int32_t)sensors->engine_rpm-(int32_t)sensors->input_rpm);
        this->last_slip_add_time = GET_CLOCK_TIME();
    }
    // See if we should be enabled in gear
    InternalTccState targ = InternalTccState::Open;
    if (
        (cmp_gear == GearboxGear::First && TCC_CURRENT_SETTINGS.enable_d1) ||
        (cmp_gear == GearboxGear::Second && TCC_CURRENT_SETTINGS.enable_d2) ||
        (cmp_gear == GearboxGear::Third && TCC_CURRENT_SETTINGS.enable_d3) ||
        (cmp_gear == GearboxGear::Fourth && TCC_CURRENT_SETTINGS.enable_d4) ||
        (cmp_gear == GearboxGear::Fifth && TCC_CURRENT_SETTINGS.enable_d5)
    ) {
        // See if we should slip or close
        if (sensors->input_rpm >= TCC_CURRENT_SETTINGS.min_locking_rpm) {
            if (sensors->pedal_pos == 0 && sensors->output_rpm >= TCC_CURRENT_SETTINGS.sailing_mode_active_rpm) {
                targ = InternalTccState::Slipping; // Gliding mode
            } else {
                int slip_now = abs(sensors->engine_rpm - sensors->input_rpm);
                int slip_avg = abs(this->slip_average->get_average());
                if (this->target_tcc_state == InternalTccState::Open && sensors->pedal_pos < 80 && slip_now < 200 && slip_avg < 200) {
                    targ = InternalTccState::Slipping;
                } else if (this->target_tcc_state > InternalTccState::Open) {
                    targ = InternalTccState::Slipping;
                }
                if (this->target_tcc_state >= InternalTccState::Slipping) {
                    // Check if we should FULLY lock
                    if (
                        (sensors->pedal_pos != 0 && ((sensors->pedal_pos*100)/250) < TCC_CURRENT_SETTINGS.locking_pedal_pos_max) || // Small pedal input (Safe to lock)
                        sensors->output_rpm > TCC_CURRENT_SETTINGS.force_lock_min_output_rpm // Force lock at very high speeds no matter the pedal position
                    ) {
                        // Only lock if we have low slip
                        if (slip_now < 50 && slip_avg < 50) {
                            targ = InternalTccState::Closed;
                        }
                    }
                }
            }
        }
    }
    if (is_shifting) {
        targ = MIN(targ, this->shift_req_tcc_state);
    }

    TccReqState engine_req_state = egs_can_hal->get_engine_tcc_override_request(500);
    if (TccReqState::None != engine_req_state) {
        // Engine is requesting at most to slip the converter
        if (TCC_CURRENT_SETTINGS.react_on_engine_slip_request && engine_req_state == TccReqState::Slipping && targ > InternalTccState::Slipping) {
            targ = InternalTccState::Slipping;
        } 
        // Engine is requesting full TCC open
        else if (TCC_CURRENT_SETTINGS.react_on_engine_open_request) {
            targ = InternalTccState::Open;
        }
    }

    this->target_tcc_state = targ;
    int rpm_delta = slip_average->get_average();

    uint32_t time_since_last_adapt = GET_CLOCK_TIME() - this->last_adapt_check;
    bool at_req_pressure = this->tcc_pressure_current == this->tcc_pressure_target;

    int load_as_percent = ((int)sensors->static_torque*100) / this->rated_max_torque;

    LookupTable* slip_table = slip_2_3;
    LookupTable* lock_table = lock_2_3;
    if (curr_gear == GearboxGear::Fourth || curr_gear == GearboxGear::Fifth) {
        slip_table = slip_4_5;
        lock_table = lock_4_5;
    }

    int adapt_write_cell_id = -1; // Invalid cell (Do not write to adaptation)
    if (load_as_percent < load_header[1]) {
        adapt_write_cell_id = 0; // For coasting
    } else if (load_as_percent < 10) {
        adapt_write_cell_id = 1; // 0-10%
    } else if (load_as_percent >= 10 && load_as_percent < 35) {
        adapt_write_cell_id = 2; // 20-30%
    } else if (load_as_percent > 35 && load_as_percent < 65) {
        adapt_write_cell_id = 3; // 40-60%
    } else if (load_as_percent > 65 && load_as_percent < 85) {
        adapt_write_cell_id = 4; // 70-80%
    } else if (load_as_percent > 85 && load_as_percent < 110) {
        adapt_write_cell_id = 5; // 90-110%
    } else if (load_as_percent > 110) {
        adapt_write_cell_id = 6; // For very high loads
    }

    if (this->target_tcc_state == InternalTccState::Open) {
        this->tcc_pressure_current = 0;
        this->tcc_pressure_target = 0;
    } else if (this->target_tcc_state == InternalTccState::Slipping) {
        if (adapt_write_cell_id != -1) {
            int16_t* slip_values = slip_table->get_current_data();
            if (at_req_pressure && rpm_delta > 100 && time_since_last_adapt > 500) {
                slip_values[adapt_write_cell_id] += 2;
                if (adapt_write_cell_id != 0) { // If not cloasting
                    // Check all upper cells and increase value if they are not increased
                    // This helps speed up adaptation of higher loads even when user may just be cruising at low load
                    for (int i = adapt_write_cell_id; i < 7; i++) {
                        slip_values[i] = MAX(slip_values[i], slip_values[adapt_write_cell_id]);
                    }
                }
                int16_t* lock_values = lock_table->get_current_data();
                lock_values[adapt_write_cell_id] = MAX(slip_values[adapt_write_cell_id], lock_values[adapt_write_cell_id]);
                this->last_adapt_check = GET_CLOCK_TIME();
            } else if (at_req_pressure && rpm_delta <= 20 && time_since_last_adapt > 500) {
                if (slip_values[adapt_write_cell_id] > 100) {
                    slip_values[adapt_write_cell_id] -= 1; 
                }
                this->last_adapt_check = GET_CLOCK_TIME();
            }
        }
        this->tcc_pressure_target = slip_table->get_value(load_as_percent);
    } else if (this->target_tcc_state == InternalTccState::Closed) {
        if (adapt_write_cell_id != -1) {
            int16_t* lock_values = lock_table->get_current_data();
            if (at_req_pressure && rpm_delta > 20 && time_since_last_adapt > 500) {
                lock_values[adapt_write_cell_id] += 2;
                if (adapt_write_cell_id != 0) { // If not cloasting
                    // Check all upper cells and increase value if they are not increased
                    // This helps speed up adaptation of higher loads even when user may just be cruising at low load
                    for (int i = adapt_write_cell_id; i < 7; i++) {
                        lock_values[i] = MAX(lock_values[i], lock_values[adapt_write_cell_id]);
                    }
                }
                this->last_adapt_check = GET_CLOCK_TIME();
            }
        }
        this->tcc_pressure_target = lock_table->get_value(load_as_percent);
    }

    if (this->target_tcc_state == this->current_tcc_state) {
        this->tcc_pressure_current = this->tcc_pressure_target;
        this->prev_state_tcc_pressure = this->tcc_pressure_current;
        this->last_state_stable_time = GET_CLOCK_TIME();
    } else if (this->target_tcc_state > this->current_tcc_state) { // Less -> More lock
        if (this->target_tcc_state == InternalTccState::Slipping) { // Open -> Slipping
            this->tcc_pressure_current = interpolate_float(
                GET_CLOCK_TIME(),
                0,
                this->tcc_pressure_target,
                this->last_state_stable_time,
                this->last_state_stable_time+200,
                InterpType::Linear
            );
        } else { // Slipping -> Closed
            this->tcc_pressure_current = interpolate_float(
                GET_CLOCK_TIME(),
                this->prev_state_tcc_pressure,
                this->tcc_pressure_target,
                this->last_state_stable_time,
                this->last_state_stable_time+100,
                InterpType::Linear
            );
        }
    } else { // More -> Less lock
        if (this->target_tcc_state == InternalTccState::Open) { // Slipping -> Open
            this->tcc_pressure_current = interpolate_float(
                GET_CLOCK_TIME(),
                this->prev_state_tcc_pressure,
                0,
                this->last_state_stable_time,
                this->last_state_stable_time+100,
                InterpType::Linear
            );
        } else { // Closed -> Slipping
            this->tcc_pressure_current = interpolate_float(
                GET_CLOCK_TIME(),
                this->prev_state_tcc_pressure,
                this->tcc_pressure_target,
                this->last_state_stable_time,
                this->last_state_stable_time+200,
                InterpType::Linear
            );
        }
    }

    // Pressure achieved.
    if (this->tcc_pressure_target == this->tcc_pressure_current) {
        this->current_tcc_state = this->target_tcc_state;
        this->prev_state_tcc_pressure = this->tcc_pressure_current;
        this->last_state_stable_time = GET_CLOCK_TIME();
    }

    //int tcc_corrected = this->tcc_pressure_current * ((float)pressure_manager->vby_settings()->working_pressure_compensation.new_min / (float)pressure_manager->get_calc_inlet_pressure());
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

void TorqueConverter::set_stationary() {
    this->was_stationary = true;
}

int16_t TorqueConverter::get_slip_filtered() {
    return this->slip_average->get_average();
}

uint8_t TorqueConverter::get_current_state() {
    return (uint8_t)this->current_tcc_state;
}

uint8_t TorqueConverter::get_target_state() {
    return (uint8_t)this->target_tcc_state;
}

uint8_t TorqueConverter::get_can_req_bits() {
    return  0;
}

uint16_t TorqueConverter::get_current_pressure() {
    return this->tcc_pressure_current;
}

uint16_t TorqueConverter::get_target_pressure() {
    return this->tcc_pressure_target;
}