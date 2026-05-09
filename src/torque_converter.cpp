#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcu_maths.h"
#include "nvs/eeprom_impl.h"
#include "nvs/all_keys.h"
#include "adapt_maps.h"
#include "maps.h"
#include "common_structs_ops.h"
#include "egs_calibration/calibration_structs.h"

#define LOAD_SIZE TCC_SLIP_ADAPT_MAP_SIZE/5

const int16_t rpm_map_x_headers[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100}; // Load %
const int16_t rpm_map_y_headers[8] = {1000, 1200, 1400, 1600, 1800, 2000, 4000, 6000}; // RPM
const uint16_t MAX_TCC_P_SAMPLE_COUNT = 100; // 2000ms
const int16_t SLIP_V_WHEN_OPEN = 100; // 100RPM is the threshold for when we start to activate the converter clutch
const int16_t SLIP_V_WHEN_LOCKED = 10; // 10RPM for locking (Means we can monitor for over locking)
const int16_t SLIP_V_OVERLOCKED = SLIP_V_WHEN_LOCKED/2;
const int16_t SLIP_V_UNDERLOCKED = SLIP_V_WHEN_LOCKED*2;
const uint8_t SLIP_SAMPLES_AVG = 25; // 500ms

TorqueConverter::TorqueConverter(uint16_t max_gb_rating)  {
    if (0 == TCC_CURRENT_SETTINGS.tcc_max_trq_override) {
        this->rated_max_torque = max_gb_rating;
    } else {
        this->rated_max_torque = TCC_CURRENT_SETTINGS.tcc_max_trq_override;
    }

    const int16_t adapt_map_x_headers[LOAD_SIZE] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 125, 150}; // Load %
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
    } else if (old > 15000) {
        old = 15000;
    }
    dest[(LOAD_SIZE*gear_int) + load_idx] = old;
    for (int i = load_idx; i < LOAD_SIZE; i++) {
        if (dest[(LOAD_SIZE*gear_int) + i] < old) {
            dest[(LOAD_SIZE*gear_int) + i] = old;
        }
    }
}

int16_t get_cell_value(int16_t* dest, GearboxGear gear, uint8_t load_idx) {
    uint8_t gear_int = (uint8_t)gear;
    if (gear_int == 0 || gear_int > 5) {
        return 0;
    } // Gear should be 1-5
    gear_int -= 1; // Convert to range 0-4 for gear
    return dest[(LOAD_SIZE*gear_int) + load_idx];
}


void TorqueConverter::update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors) {
    int slip_now = abs((int32_t)sensors->engine_rpm-(int32_t)sensors->input_rpm);
    int motor_torque = sensors->converted_torque;
    int load_as_percent = abs(((int)motor_torque*100) / this->rated_max_torque);
    this->engine_load_percent = load_as_percent;

    // Adapt sample size based on ATF temp
    // this way, as the ATF warms up, simulated response time
    // decreases
    uint16_t pressure_samples = interpolate_float(
        sensors->atf_temp,
        MAX_TCC_P_SAMPLE_COUNT,
        MAX_TCC_P_SAMPLE_COUNT/2,
        -10,
        70,
        InterpType::Linear
    );

    // Conditions for no TCC
    if (
        !this->tcc_solenoid_enabled || // Diagnostic request
        !init_tables_ok // Some data was not initialized or invalid
    ) {
        this->tcc_commanded_pressure = 0;
        pm->set_target_tcc_pressure(this->tcc_commanded_pressure);
        this->current_tcc_state = InternalTccState::Open;
        this->target_tcc_state = InternalTccState::Open;
        this->slip_target = SLIP_V_WHEN_OPEN;
        this->prefill_done = false;
        this->prefill_running = false;
        return;
    }

    if (this->tcc_actual_pressure/100 > this->tcc_commanded_pressure) {
        // Drop in pressure
        this->tcc_actual_pressure = first_order_filter(pressure_samples/4, this->tcc_commanded_pressure*100, this->tcc_actual_pressure);
    } else {
        // Increasing in pressure
        this->tcc_actual_pressure = first_order_filter(pressure_samples, this->tcc_commanded_pressure*100, this->tcc_actual_pressure);
    }
    if (abs(this->tcc_actual_pressure/100-this->tcc_commanded_pressure)<2) {
        this->tcc_actual_pressure = this->tcc_commanded_pressure*100;
    }
    
    GearboxGear cmp_gear = curr_gear;

    this->tcc_slip_filtered = first_order_filter(SLIP_SAMPLES_AVG, slip_now*100, this->tcc_slip_filtered);
    // See if we should be enabled in gear
    InternalTccState targ = InternalTccState::Open;
    int slipping_rpm_targ = SLIP_V_WHEN_OPEN;

    bool can_enable_tcc = sensors->atf_temp > -10 && sensors->input_rpm > rpm_map_y_headers[0];

    if (
        can_enable_tcc &&
        ((cmp_gear == GearboxGear::First && TCC_CURRENT_SETTINGS.enable_d1)||
        (cmp_gear == GearboxGear::Second && TCC_CURRENT_SETTINGS.enable_d2)||
        (cmp_gear == GearboxGear::Third && TCC_CURRENT_SETTINGS.enable_d3) ||
        (cmp_gear == GearboxGear::Fourth && TCC_CURRENT_SETTINGS.enable_d4)||
        (cmp_gear == GearboxGear::Fifth && TCC_CURRENT_SETTINGS.enable_d5))
    ) {
        // See if we should slip or close based on maps
        targ = InternalTccState::Open;
        int pedal_as_percent = (sensors->pedal_pos*100)/250;
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
        if (is_shifting) {
            // Check previous target
            bool open_tcc = false;
            if (this->target_tcc_state == InternalTccState::Open) {
                // Prevent lockup during shift
                open_tcc = true;
            } else {
                if (upshifting) {
                    if (sensors->pedal_pos > 15) {
                        open_tcc = TCC_CURRENT_SETTINGS.unlock_load_upshifts;
                    } else {
                        open_tcc = TCC_CURRENT_SETTINGS.unlock_coasting_upshifts;
                    }
                } else {
                    // Downshifting
                    if (sensors->pedal_pos > 15) {
                        open_tcc = TCC_CURRENT_SETTINGS.unlock_load_downshifts;
                    } else {
                        open_tcc = TCC_CURRENT_SETTINGS.unlock_coasting_downshifts;
                    }
                }
            }
            if (open_tcc) {
                targ = InternalTccState::Open;
                slipping_rpm_targ = MAX(this->slip_target, SLIP_V_WHEN_OPEN);
            } else {
                // Otherwise prevent increase in TCC State
                targ = MIN(this->target_tcc_state, targ);
                slipping_rpm_targ = MAX(this->slip_target, slipping_rpm_targ);
            }
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
    this->slip_target = MIN(slipping_rpm_targ, SLIP_V_WHEN_OPEN);

    this->engine_output_joule = sensors->engine_rpm * (abs(motor_torque)) / 9.5488;
    if (likely(sensors->engine_rpm >= sensors->input_rpm)) {
        float rpm_as_percent = (float)sensors->input_rpm / (float)sensors->engine_rpm;
        this->absorbed_power_joule = this->engine_output_joule - (this->engine_output_joule * rpm_as_percent);
    } else {
        this->absorbed_power_joule = 0;
    }

    bool is_adaptable = abs(this->tcc_commanded_pressure-this->tcc_actual_pressure/100) < 2;
    if (!TCC_CURRENT_SETTINGS.adapt_enable) {
        is_adaptable = false;
    }
    if (sensors->atf_temp < TCC_CURRENT_SETTINGS.tcc_temp_multiplier.raw_max) {
        is_adaptable = false;
    }
    // Disable adapting when coasting (Some load doesn't map 1:1)
    if (motor_torque < 0) {
        is_adaptable = false;
    }
    uint8_t load_cell = 0xFF; // Invalid cell (Do not write to adaptation)
    if (!is_shifting){
        // 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 125, 150
        if (load_as_percent <= 10) {
            load_cell = 0;
        } else if (load_as_percent <= 20) {
            load_cell = 1;
        } else if (load_as_percent <= 30) {
            load_cell = 2;
        } else if (load_as_percent <= 40) {
            load_cell = 3;
        } else if (load_as_percent <= 50) {
            load_cell = 4;
        } else if (load_as_percent <= 60) {
            load_cell = 5;
        } else if (load_as_percent <= 70) {
            load_cell = 6;
        } else if (load_as_percent <= 80) {
            load_cell = 7;
        } else if (load_as_percent <= 90) {
            load_cell = 8;
        } else if (load_as_percent <= 100) {
            load_cell = 9;
        } else if (load_as_percent <= 125) {
            load_cell = 10;
        } else if (load_as_percent <= 150) {
            load_cell = 11;
        } else {
            load_cell = 12;
        }
    }
    // Prefilling when suddenly increasing state requested
    if (this->target_tcc_state > this->current_tcc_state && this->tcc_actual_pressure/100 < 100 && !prefill_done) {
        // Prefill logic
        if (false == prefill_running) {
            // Var setup (Start of prefill)
            prefill_running = true;
            prefill_cycles = TCC_CURRENT_SETTINGS.prefill_cycles*3;
        }

        if (prefill_cycles > 0) {
            prefill_cycles -= 1;
        }
        if (prefill_cycles == 0 || slip_now <= this->slip_target) {
            prefill_done = true;
        }
        this->tcc_commanded_pressure = TCC_CURRENT_SETTINGS.prefill_pressure;
    } else {
        int slip_adaptation = abs(this->tcc_slip_filtered/100);
        // Constant logic
        if (this->target_tcc_state == InternalTccState::Open) {
            this->tcc_commanded_pressure = 0;
        } else if (this->target_tcc_state == InternalTccState::Slipping) {
            if (is_adaptable && this->target_tcc_state == this->current_tcc_state) {
                int slip_min = MAX(slip_target * 0.8, SLIP_V_UNDERLOCKED);
                if (load_cell != 0xFF && slip_adaptation > slip_target) {
                    int adder = interpolate_float(slip_adaptation, 1, 100, slip_target, slip_target*2, InterpType::Linear);
                    set_adapt_cell(this->tcc_slip_map->get_current_data(), curr_gear, load_cell, adder);
                    // Adjust the locking map too if we are increasing slipping pressure
                    int16_t slip_v = get_cell_value(this->tcc_slip_map->get_current_data(), curr_gear, load_cell);
                    int16_t lock_v = get_cell_value(this->tcc_lock_map->get_current_data(), curr_gear, load_cell);
                    if (slip_v > lock_v+100) {
                        int delta = slip_v-lock_v;
                        set_adapt_cell(this->tcc_lock_map->get_current_data(), curr_gear, load_cell, delta);
                    }
                } else if (load_cell != 0xFF && slip_adaptation <= slip_min) {
                    int remover = interpolate_float(slip_adaptation, -10, -1, slip_min/2, slip_min, InterpType::Linear);
                    set_adapt_cell(this->tcc_slip_map->get_current_data(), curr_gear, load_cell, remover);
                }
            }
            this->tcc_commanded_pressure = this->tcc_slip_map->get_value(load_as_percent, (uint8_t)curr_gear);
        } else if (this->target_tcc_state == InternalTccState::Closed) {
            // Closed state now is 10RPM or less delta (Original EGS) (up to +/-10 RPM either way is allowed (so 0-20RPM delta))
            if (is_adaptable && this->target_tcc_state == this->current_tcc_state) {
                if  (load_cell != 0xFF && SLIP_V_UNDERLOCKED < slip_adaptation) {
                    int adder = interpolate_float(slip_adaptation, 1, 50, SLIP_V_UNDERLOCKED, SLIP_V_UNDERLOCKED*2, InterpType::Linear);
                    set_adapt_cell(this->tcc_lock_map->get_current_data(), curr_gear, load_cell, adder);
                } else if (load_cell != 0xFF && SLIP_V_OVERLOCKED > slip_adaptation) {
                    int remover = interpolate_float(slip_adaptation, -5, 0, 0, SLIP_V_OVERLOCKED, InterpType::Linear);
                    // RPM Delta is too small, meaning we are over-locking, we can reduce the pressure a bit
                    set_adapt_cell(this->tcc_lock_map->get_current_data(), curr_gear, load_cell, remover);

                    // Adjust the slipping map too if we are decreasing slipping pressure
                    int16_t slip_v = get_cell_value(this->tcc_slip_map->get_current_data(), curr_gear, load_cell);
                    int16_t lock_v = get_cell_value(this->tcc_lock_map->get_current_data(), curr_gear, load_cell);
                    if (lock_v < slip_v-500) {
                        int delta = lock_v-slip_v;
                        set_adapt_cell(this->tcc_slip_map->get_current_data(), curr_gear, load_cell, delta);
                    }
                }
            }
            this->tcc_commanded_pressure = this->tcc_lock_map->get_value(load_as_percent, (uint8_t)curr_gear);
        }
    }
    if (!is_shifting && was_shifting) {
        was_shifting = false;
    }

    // Pressure achieved.
    if (abs(this->tcc_commanded_pressure-this->tcc_actual_pressure/100) < 1) {
        this->current_tcc_state = this->target_tcc_state;
    }
    // Reset prefill info when we go to 0 pressure
    if (this->tcc_commanded_pressure == 0) {
        this->prefill_done = false;
        this->prefill_running = false;
    }
    // OEM EGS - Below 60C, TCC pressure is reduced by a factor based on
    // ATF temperature
    if (sensors->atf_temp < TCC_CURRENT_SETTINGS.tcc_temp_multiplier.raw_max) {
        float mul = interpolate_float(sensors->atf_temp, &TCC_CURRENT_SETTINGS.tcc_temp_multiplier, InterpType::Linear);
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
    return this->tcc_slip_filtered/100;
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
    return this->tcc_actual_pressure/100;
}

uint16_t TorqueConverter::get_target_pressure() {
    return this->tcc_commanded_pressure;
}

void TorqueConverter::shift_start(bool upshift, bool release_shifting) {
    this->is_shifting = true;
    this->was_shifting = true;
    this->release_shifting = release_shifting;
    this->upshifting = upshift;
}
void TorqueConverter::shift_end() {
    this->is_shifting = false;
}