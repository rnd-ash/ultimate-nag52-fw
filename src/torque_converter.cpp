#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcu_maths.h"
#include "nvs/eeprom_impl.h"
#include "nvs/all_keys.h"
#include "adapt_maps.h"
#include "maps.h"
#include "common_structs_ops.h"
#include "egs_calibration/calibration_structs.h"

#define LOAD_SIZE 11

const int16_t rpm_map_x_headers[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100}; // Load %
const int16_t rpm_map_y_headers[8] = {1000, 1200, 1400, 1600, 1800, 2000, 4000, 6000}; // RPM
const uint16_t MAX_TCC_P_SAMPLE_COUNT = 100; // 2000ms
const int16_t SLIP_V_WHEN_OPEN = 100; // 100RPM is the threshold for when we start to activate the converter clutch
const int16_t SLIP_V_WHEN_LOCKED = 10; // 10RPM for locking (Means we can monitor for over locking)
const int16_t SLIP_V_OVERLOCKED = SLIP_V_WHEN_LOCKED/2;
const int16_t SLIP_V_UNDERLOCKED = SLIP_V_WHEN_LOCKED*2;

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

    this->slip_rpm_target_map = new StoredMap(NVS_KEY_TCC_SLIP_TARGET_MAP, TCC_RPM_TARGET_MAP_SIZE, rpm_map_x_headers, rpm_map_y_headers, 11, 8, TCC_RPM_TARGET_MAP);
    if (this->slip_rpm_target_map->init_status() != ESP_OK) {
        delete[] this->slip_rpm_target_map;
    }

    this->init_tables_ok = (this->tcc_slip_map != nullptr) && (this->tcc_lock_map != nullptr) && (this->slip_rpm_target_map != nullptr);
    if (!init_tables_ok) {
        ESP_LOGE("TCC", "Adaptation table(s) for TCC failed to load. TCC will be non functional");
    } else {
        this->slip_average = new FirstOrderAverage(10); // 200ms
        this->tcc_actual_pressure_calc = new FirstOrderAverage(MAX_TCC_P_SAMPLE_COUNT); // 500ms
        this->init_tables_ok = (this->slip_average != nullptr) && (this->tcc_actual_pressure_calc != nullptr);
        if (!init_tables_ok) {
            ESP_LOGE("TCC", "Moving avg filters for TCC failed to be allocated. TCC will be non functional");
        }
    }
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
    // Auto increase successive cells (Stops spiking)
    // > 2 so that we don't adapt negative cells
    if (load_idx > 2 && load_idx < 10) {
        for (int i = load_idx; i <= 10; i++) {
            if (dest[(LOAD_SIZE*gear_int) + i] < old) {
                dest[(LOAD_SIZE*gear_int) + i] = old;
            }
        }
    }
}


void TorqueConverter::update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors) {
    int slip_now = abs((int32_t)sensors->engine_rpm-(int32_t)sensors->input_rpm);
    int motor_torque = sensors->converted_torque;
    int load_as_percent = ((int)motor_torque*100) / this->rated_max_torque;
    this->engine_load_percent = load_as_percent;

    // Conditions for no TCC
    if (
        !this->tcc_solenoid_enabled || // Diagnostic request
        !init_tables_ok || // Some data was not initialized or invalid
        sensors->input_rpm < rpm_map_y_headers[0] || // Too low input RPM
        sensors->atf_temp < -10 // ATF is too cold (OEM EGS behaviour)
    ) {
        this->tcc_commanded_pressure = 0;
        if (init_tables_ok) { // Only if we have valid maps here
            this->tcc_actual_pressure_calc->add_sample(0);
            this->tcc_actual_pressure = this->tcc_actual_pressure_calc->get_average();
            this->slip_average->add_sample(slip_now);
        }
        pm->set_target_tcc_pressure(this->tcc_commanded_pressure);
        this->current_tcc_state = InternalTccState::Open;
        this->target_tcc_state = InternalTccState::Open;
        this->slip_target = SLIP_V_WHEN_OPEN;
        this->last_state_stable_time = GET_CLOCK_TIME();
        return;
    }
        
    // Adapt sample size based on ATF temp
    // this way, as the ATF warms up, simulated response time
    // decreases
    uint16_t samples = interpolate_float(
        sensors->atf_temp,
        MAX_TCC_P_SAMPLE_COUNT,
        MAX_TCC_P_SAMPLE_COUNT/2,
        -10,
        70,
        InterpType::Linear
    );

    this->tcc_actual_pressure_calc->set_sample_size(samples);
    this->tcc_actual_pressure_calc->add_sample(this->tcc_commanded_pressure);
    this->tcc_actual_pressure = this->tcc_actual_pressure_calc->get_average();
    
    GearboxGear cmp_gear = curr_gear;
    this->slip_average->add_sample(slip_now);
    // See if we should be enabled in gear
    InternalTccState targ = InternalTccState::Open;
    int slipping_rpm_targ = SLIP_V_WHEN_OPEN;

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
        slipping_rpm_targ = this->slip_rpm_target_map->get_value(pedal_as_percent, sensors->input_rpm);
        // Can we slip?
        if (SLIP_V_WHEN_OPEN > slipping_rpm_targ) {
            targ = InternalTccState::Slipping;
            // Can we lock?
            if (SLIP_V_WHEN_LOCKED >= slipping_rpm_targ) {
                targ = InternalTccState::Closed;
                slipping_rpm_targ = MAX(SLIP_V_WHEN_LOCKED, slipping_rpm_targ);
            }
        }
        slipping_rpm_targ = slipping_rpm_targ;

        if (is_shifting && !upshifting) {
            targ = InternalTccState::Open;
            slipping_rpm_targ = MAX(this->slip_target, SLIP_V_WHEN_OPEN);
        }
    }

    TccReqState engine_req_state = egs_can_hal->get_engine_tcc_override_request(500);
    if (TccReqState::None != engine_req_state) {
        // Engine is requesting at most to slip the converter
        if (TCC_CURRENT_SETTINGS.react_on_engine_slip_request && engine_req_state == TccReqState::Slipping && targ > InternalTccState::Slipping) {
            targ = InternalTccState::Slipping;
            slipping_rpm_targ = MAX(this->slip_target, 50);
        } 
        // Engine is requesting full TCC open
        else if (TCC_CURRENT_SETTINGS.react_on_engine_open_request) {
            targ = InternalTccState::Open;
            slipping_rpm_targ = SLIP_V_WHEN_OPEN;
        }
    }
    // Variable set
    this->target_tcc_state = targ;
    this->slip_target = slipping_rpm_targ;

    this->engine_output_joule = sensors->engine_rpm * (abs(motor_torque)) / 9.5488;
    if (likely(sensors->engine_rpm >= sensors->input_rpm)) {
        float rpm_as_percent = (float)sensors->input_rpm / (float)sensors->engine_rpm;
        this->absorbed_power_joule = this->engine_output_joule - (this->engine_output_joule * rpm_as_percent);
    } else {
        this->absorbed_power_joule = 0;
    }
    int rpm_delta = slip_average->get_average();

    uint32_t time_since_last_adapt = GET_CLOCK_TIME() - this->last_adapt_check;
    bool at_req_pressure = this->tcc_actual_pressure == this->tcc_commanded_pressure;
    int pedal_delta = sensors->pedal_delta->get_average();
    bool is_stable = abs(pedal_delta) <= 25 && abs(slip_average->get_average() - slip_now) < 10; // 10% difference allowed in our time window
    if (sensors->atf_temp < TCC_CURRENT_SETTINGS.temp_threshold_adapt) {
        // Disable adaptation below this temp threshold since
        // ATF viscocity is not lo enough for accurate adaptation
        is_stable = false;
    }
    int load_cell = -1; // Invalid cell (Do not write to adaptation)
    if (!is_shifting && time_since_last_adapt > TCC_CURRENT_SETTINGS.adapt_test_interval_ms && sensors->pedal_pos > 0){ 
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
        } else if (load_as_percent > 75 && load_as_percent <= 100) {
            load_cell = 7;
        } else if (load_as_percent > 100 && load_as_percent <= 125) {
            load_cell = 8;
        } else if (load_as_percent > 125 && load_as_percent <= 140) {
            load_cell = 9;
        } else if (load_as_percent > 140) {
            load_cell = 10;
        }
    }

    if (this->target_tcc_state == InternalTccState::Open) {
        this->tcc_commanded_pressure = 0;
    } else if (this->current_tcc_state == InternalTccState::Slipping) {
        if (load_cell != -1) {
            if (at_req_pressure && is_stable && abs(rpm_delta) > slip_target * 1.1) {
                set_adapt_cell(this->tcc_slip_map->get_current_data(), curr_gear, load_cell, +5);
                this->last_adapt_check = GET_CLOCK_TIME();
            } else if (at_req_pressure && is_stable && abs(rpm_delta) <= slip_target * 0.9) {
                set_adapt_cell(this->tcc_slip_map->get_current_data(), curr_gear, load_cell, -1);
                this->last_adapt_check = GET_CLOCK_TIME();
            }
        }
        this->tcc_commanded_pressure = this->tcc_slip_map->get_value(load_as_percent, (uint8_t)curr_gear);
    } else if (this->current_tcc_state == InternalTccState::Closed) {
        // Closed state now is 10RPM or less delta (Original EGS) (up to +/-10 RPM either way is allowed (so 0-20RPM delta))
        if (is_stable && load_cell != -1 && at_req_pressure) {
            if  (SLIP_V_UNDERLOCKED < rpm_delta) {
                set_adapt_cell(this->tcc_lock_map->get_current_data(), curr_gear, load_cell, +5);
                this->last_adapt_check = GET_CLOCK_TIME();
            } else if (SLIP_V_OVERLOCKED > rpm_delta) {
                // RPM Delta is too small, meaning we are over-locking, we can reduce the pressure a bit
                set_adapt_cell(this->tcc_lock_map->get_current_data(), curr_gear, load_cell, -5);
                this->last_adapt_check = GET_CLOCK_TIME();
            }
        }
        this->tcc_commanded_pressure = this->tcc_lock_map->get_value(load_as_percent, (uint8_t)curr_gear);
    }
    if (!is_shifting && was_shifting) {
        was_shifting = false;
    }

    // Pressure achieved.
    if (at_req_pressure) {
        this->current_tcc_state = this->target_tcc_state;
        this->last_state_stable_time = GET_CLOCK_TIME();
    }
    // OEM EGS - Below 60C, TCC pressure is reduced by a factor based on
    // ATF temperature
    if (sensors->atf_temp < 70) {
        float mul = interpolate_float(sensors->atf_temp, 0.6, 1.0, -10, 70, InterpType::Linear);
        this->tcc_commanded_pressure = (float)this->tcc_commanded_pressure*mul;
    }
    pm->set_target_tcc_pressure(this->tcc_commanded_pressure);
}

InternalTccState TorqueConverter::__get_internal_state(void) {
    return this->current_tcc_state;
}

TccClutchStatus TorqueConverter::get_clutch_state(void) {
    TccClutchStatus ret = TccClutchStatus::Open;
    InternalTccState targ = this->target_tcc_state;
    // Reduction or equal state (EG: Closed -> Slipping)
    // Just return the target state
    if (this->current_tcc_state >= targ) {
        switch (this->current_tcc_state) {
            case InternalTccState::Closed:
                ret = TccClutchStatus::Closed;
                break;
            case InternalTccState::Slipping:
                ret = TccClutchStatus::Slipping;
                break;
            case InternalTccState::Open: // Already set
            default:
                ret = TccClutchStatus::Open;
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
    return this->tcc_actual_pressure;
}

uint16_t TorqueConverter::get_target_pressure() {
    return this->tcc_commanded_pressure;
}

void TorqueConverter::shift_start(bool upshift) {
    this->is_shifting = true;
    this->was_shifting = true;
    this->upshifting = upshift;
}
void TorqueConverter::shift_end() {
    this->is_shifting = false;
}