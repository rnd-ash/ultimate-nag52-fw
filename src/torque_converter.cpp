#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcu_maths.h"
#include "nvs/eeprom_impl.h"
#include "nvs/all_keys.h"
#include "adapt_maps.h"
#include "maps.h"

#define LOAD_SIZE 11

TorqueConverter::TorqueConverter(uint16_t max_gb_rating)  {
    this->rated_max_torque = max_gb_rating;

    const int16_t adapt_map_x_headers[LOAD_SIZE] = {-25, 0, 10, 20, 30, 40, 50, 75, 100, 125, 150}; // Load %
    const int16_t adapt_map_y_headers[5] = {1,2,3,4,5}; // Gear
    this->tcc_slip_map = new StoredMap(NVS_KEY_TCC_ADAPT_SLIP_MAP, TCC_SLIP_ADAPT_MAP_SIZE, adapt_map_x_headers, adapt_map_y_headers, LOAD_SIZE, 5, TCC_SLIP_ADAPT_MAP);
    if (this->tcc_slip_map->init_status() != ESP_OK) {
        delete[] this->tcc_slip_map;
    }

    this->tcc_lock_map = new StoredMap(NVS_KEY_TCC_ADAPT_LOCK_MAP, TCC_SLIP_ADAPT_MAP_SIZE, adapt_map_x_headers, adapt_map_y_headers, LOAD_SIZE, 5, TCC_LOCK_ADAPT_MAP);
    if (this->tcc_lock_map->init_status() != ESP_OK) {
        delete[] this->tcc_lock_map;
    }

    const int16_t rpm_map_x_headers[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100}; // Load %
    const int16_t rpm_map_y_headers[11] = {1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000}; // RPM
    this->slip_rpm_target_map = new StoredMap(NVS_KEY_TCC_SLIP_TARGET_MAP, TCC_RPM_TARGET_MAP_SIZE, rpm_map_x_headers, rpm_map_y_headers, 11, 11, TCC_RPM_TARGET_MAP);
    if (this->slip_rpm_target_map->init_status() != ESP_OK) {
        delete[] this->slip_rpm_target_map;
    }

    this->init_tables_ok = (this->tcc_slip_map != nullptr) && (this->tcc_lock_map != nullptr) && (this->slip_rpm_target_map != nullptr);
    if (!init_tables_ok) {
        ESP_LOGE("TCC", "Adaptation table(s) for TCC failed to load. TCC will be non functional");
    }

    this->slip_average = new MovingAverage<uint32_t>(50); // 20ms div * 50 = 1 second moving average
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

void set_adapt_cell(int16_t* dest, GearboxGear gear, uint8_t load_idx, int16_t offset) {
    // Y is gear
    // X is load cell
    uint8_t gear_int = (uint8_t)gear;
    if (gear_int == 0 || gear_int > 5) {return;} // Gear should be 1-5
    gear_int -= 1; // Convert to range 0-4 for gear
    int16_t old = dest[(LOAD_SIZE*gear_int) + load_idx];
    old += offset;
    if (old < 100) {
        old = 100;
    }
    dest[(LOAD_SIZE*gear_int) + load_idx] = old;
}

void max_adapt_row(int16_t* dest, GearboxGear gear, uint8_t load_idx_start) {
    uint8_t gear_int = (uint8_t)gear;
    if (gear_int == 0 || gear_int > 5) {return;} // Gear should be 1-5
    int16_t at_cell = dest[(LOAD_SIZE*gear_int) + load_idx_start];
    for (int i = load_idx_start; i < LOAD_SIZE; i++) {
        dest[(LOAD_SIZE*gear_int) + i] = MAX(at_cell, dest[(LOAD_SIZE*gear_int) + i]);
    }
}

void TorqueConverter::update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors) {
    // TCC is commanded to be off,
    // or adaptation table failure.
    if (!this->tcc_solenoid_enabled || !init_tables_ok) {
        pm->set_target_tcc_pressure(0);
        return;
    }
    
    GearboxGear cmp_gear = curr_gear;
    this->slip_average->add_sample((int32_t)sensors->engine_rpm-(int32_t)sensors->input_rpm);
    // See if we should be enabled in gear
    InternalTccState targ = InternalTccState::Open;
    if (
        (cmp_gear == GearboxGear::First && TCC_CURRENT_SETTINGS.enable_d1) ||
        (cmp_gear == GearboxGear::Second && TCC_CURRENT_SETTINGS.enable_d2) ||
        (cmp_gear == GearboxGear::Third && TCC_CURRENT_SETTINGS.enable_d3) ||
        (cmp_gear == GearboxGear::Fourth && TCC_CURRENT_SETTINGS.enable_d4) ||
        (cmp_gear == GearboxGear::Fifth && TCC_CURRENT_SETTINGS.enable_d5)
    ) {
        // See if we should slip or close based on maps
        targ = InternalTccState::Open;
        int pedal_as_percent = (sensors->pedal_smoothed->get_average()*100)/250;
        int slipping_rpm_targ = this->slip_rpm_target_map->get_value(pedal_as_percent, sensors->input_rpm);
        this->slip_target = slipping_rpm_targ; // For diag data
        // Can we slip?
        if (slipping_rpm_targ <= 100) {
            targ = InternalTccState::Slipping;
            // Can we lock?
            if (slipping_rpm_targ <= 10) {
                targ = InternalTccState::Closed;
            }
        }
        // Override locking (Only if slipping is allowed and at high RPM)
        if (this->target_tcc_state >= InternalTccState::Slipping && sensors->output_rpm > TCC_CURRENT_SETTINGS.force_lock_min_output_rpm && pedal_as_percent != 0) {
                targ = InternalTccState::Closed;
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
    int pedal_delta = sensors->pedal_smoothed->front() - sensors->pedal_smoothed->back();
    bool is_stable = abs(pedal_delta) <= 25; // 10% difference allowed in our time window

    int load_as_percent = ((int)sensors->static_torque*100) / this->rated_max_torque;
    int load_cell = -1; // Invalid cell (Do not write to adaptation)
    if (time_since_last_adapt > 500){ 
        // -25, 0, 10, 20, 30, 40, 50, 75, 100, 125, 150
        if (load_as_percent < -5) {
            load_cell = 0;
        } else if (load_as_percent > -5 && load_as_percent <= 5) {
            load_cell = 1;
        } else if (load_as_percent > 5 && load_as_percent <= 15) {
            load_cell = 2;
        } else if (load_as_percent > 15 && load_as_percent <= 25) {
            load_cell = 3;
        } else if (load_as_percent > 25 && load_as_percent <= 35) {
            load_cell = 4;
        } else if (load_as_percent > 35 && load_as_percent <= 45) {
            load_cell = 5;
        } else if (load_as_percent > 45 && load_as_percent <= 55) {
            load_cell = 6;
        } else if (load_as_percent > 65 && load_as_percent <= 85) {
            load_cell = 7;
        } else if (load_as_percent > 90 && load_as_percent <= 112) {
            load_cell = 8;
        } else if (load_as_percent > 112 && load_as_percent <= 135) {
            load_cell = 9;
        } else if (load_as_percent > 140) {
            load_cell = 10;
        }
    }

    if (this->target_tcc_state == InternalTccState::Open) {
        this->tcc_pressure_current = 0;
        this->tcc_pressure_target = 0;
    } else if (this->target_tcc_state == InternalTccState::Slipping) {
        if (load_cell != -1) {
            if (at_req_pressure && is_stable && abs(rpm_delta) > slip_target * 1.1) {
                set_adapt_cell(this->tcc_slip_map->get_current_data(), curr_gear, load_cell, +2);
                if (load_cell != 0) { // If not coasting
                    // Check all upper cells and increase value if they are not increased
                    // This helps speed up adaptation of higher loads even when user may just be cruising at low load
                    max_adapt_row(this->tcc_slip_map->get_current_data(), curr_gear, load_cell);
                }
                this->last_adapt_check = GET_CLOCK_TIME();
            } else if (at_req_pressure && is_stable && abs(rpm_delta) <= slip_target * 0.9) {
                set_adapt_cell(this->tcc_slip_map->get_current_data(), curr_gear, load_cell, -1);
                this->last_adapt_check = GET_CLOCK_TIME();
            }
        }
        this->tcc_pressure_target = this->tcc_slip_map->get_value(load_as_percent, (uint8_t)curr_gear);
    } else if (this->target_tcc_state == InternalTccState::Closed) {
        if (is_stable && load_cell != -1 && at_req_pressure && rpm_delta > 10) {
            set_adapt_cell(this->tcc_lock_map->get_current_data(), curr_gear, load_cell, +2);
            if (load_cell != 0) { // If not coasting
                // Check all upper cells and increase value if they are not increased
                // This helps speed up adaptation of higher loads even when user may just be cruising at low load
                max_adapt_row(this->tcc_lock_map->get_current_data(), curr_gear, load_cell);
            }
            this->last_adapt_check = GET_CLOCK_TIME();
        }
        this->tcc_pressure_target = this->tcc_lock_map->get_value(load_as_percent, (uint8_t)curr_gear);
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
                this->last_state_stable_time+500,
                InterpType::Linear
            );
        } else { // Slipping -> Closed
            this->tcc_pressure_current = interpolate_float(
                GET_CLOCK_TIME(),
                this->prev_state_tcc_pressure,
                this->tcc_pressure_target,
                this->last_state_stable_time,
                this->last_state_stable_time+1000,
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
                this->last_state_stable_time+500,
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