#include "torque_converter.h"
#include "canbus/can_defines.h"
#include "canbus/can_hal.h"
#include "nvs/eeprom_config.h"
#include "nvs/module_settings.h"
#include "solenoids/solenoids.h"
#include "stored_map.h"
#include "tcu_maths.h"
#include "tcu_maths_impl.h"
#include "nvs/eeprom_impl.h"
#include "nvs/all_keys.h"
#include "adapt_maps.h"
#include "maps.h"
#include "common_structs_ops.h"
#include "egs_calibration/calibration_structs.h"
#include <cstdint>

#define LOAD_SIZE TCC_SLIP_ADAPT_MAP_SIZE/5

const int16_t rpm_map_x_headers[11] = {0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 100}; // Load %
const int16_t rpm_map_y_headers[8] = {1000, 1200, 1400, 1600, 1800, 2000, 4000, 6000}; // RPM
const uint16_t MAX_TCC_P_SAMPLE_COUNT = 20; // 400ms
const int16_t SLIP_V_WHEN_OPEN = 100; // 100RPM is the threshold for when we start to activate the converter clutch
const int16_t SLIP_V_WHEN_LOCKED = 10; // 10RPM for locking (Means we can monitor for over locking)
const int16_t SLIP_V_OVERLOCKED = SLIP_V_WHEN_LOCKED/2;
const int16_t SLIP_V_UNDERLOCKED = SLIP_V_WHEN_LOCKED*2;
const uint8_t SLIP_SAMPLES_AVG = 25; // 500ms

const uint16_t SLIP_X_COAST[5] = {1000, 1500, 3500, 4000, 6000};
const uint16_t SLIP_Z_COAST[5] = {  70,   40,   40,   10,   10};

const uint16_t TCC_P_ADDER_X[5] = {600, 750,  990, 1500, 6000};
const uint16_t TCC_P_ADDER_Z[5] = {  0,  50,  100,  200,  200};

const int16_t TCC_PID_X[5] = {-20, 0, 100, 250, 500};
const int16_t TCC_PID_PZ_PULL[5] = {20, 10, 0, 0, 0};
const int16_t TCC_PID_IZ_PULL[5] = {300, 200, 100, 30, 3};

const int16_t TCC_PID_PZ_PUSH[5] = {50, 50, 75, 100, 125};
const int16_t TCC_PID_IZ_PUSH[5] = {300, 20, 5, 2, 2};

TorqueConverter::TorqueConverter(uint16_t max_gb_rating)  {
    if (0 == TCC_CURRENT_SETTINGS.tcc_max_trq_override) {
        this->rated_max_torque = max_gb_rating;
    } else {
        this->rated_max_torque = TCC_CURRENT_SETTINGS.tcc_max_trq_override;
    }

    this->tcc_adapt_map_d1 = new StoredMap(NVS_KEY_TCC_ADAPT_MAP_1, TCC_ADAPT_MAP_Z_SIZE, TCC_ADAPT_MAP_X, TCC_ADAPT_MAP_Y, 6, 6, TCC_ADAPT_MAP_Z);
    if (this->tcc_adapt_map_d1->init_status() != ESP_OK) {
        delete this->tcc_adapt_map_d1;
        this->tcc_adapt_map_d1 = nullptr;
    }

    this->tcc_adapt_map_d2 = new StoredMap(NVS_KEY_TCC_ADAPT_MAP_2, TCC_ADAPT_MAP_Z_SIZE, TCC_ADAPT_MAP_X, TCC_ADAPT_MAP_Y, 6, 6, TCC_ADAPT_MAP_Z);
    if (this->tcc_adapt_map_d2->init_status() != ESP_OK) {
        delete this->tcc_adapt_map_d2;
        this->tcc_adapt_map_d2 = nullptr;
    }

    this->tcc_adapt_map_d3 = new StoredMap(NVS_KEY_TCC_ADAPT_MAP_3, TCC_ADAPT_MAP_Z_SIZE, TCC_ADAPT_MAP_X, TCC_ADAPT_MAP_Y, 6, 6, TCC_ADAPT_MAP_Z);
    if (this->tcc_adapt_map_d3->init_status() != ESP_OK) {
        delete this->tcc_adapt_map_d3;
        this->tcc_adapt_map_d3 = nullptr;
    }

    this->tcc_adapt_map_d4 = new StoredMap(NVS_KEY_TCC_ADAPT_MAP_4, TCC_ADAPT_MAP_Z_SIZE, TCC_ADAPT_MAP_X, TCC_ADAPT_MAP_Y, 6, 6, TCC_ADAPT_MAP_Z);
    if (this->tcc_adapt_map_d4->init_status() != ESP_OK) {
        delete this->tcc_adapt_map_d4;
        this->tcc_adapt_map_d4 = nullptr;
    }

    this->tcc_adapt_map_d5 = new StoredMap(NVS_KEY_TCC_ADAPT_MAP_5, TCC_ADAPT_MAP_Z_SIZE, TCC_ADAPT_MAP_X, TCC_ADAPT_MAP_Y, 6, 6, TCC_ADAPT_MAP_Z);
    if (this->tcc_adapt_map_d5->init_status() != ESP_OK) {
        delete this->tcc_adapt_map_d5;
        this->tcc_adapt_map_d5 = nullptr;
    }



    this->slip_rpm_target_map = new StoredMap(NVS_KEY_TCC_SLIP_TARGET_MAP, TCC_RPM_TARGET_MAP_SIZE, rpm_map_x_headers, rpm_map_y_headers, 11, 8, TCC_RPM_TARGET_MAP);
    if (this->slip_rpm_target_map->init_status() != ESP_OK) {
        delete this->slip_rpm_target_map;
        this->slip_rpm_target_map = nullptr;
    }

    this->init_tables_ok =
        (this->tcc_adapt_map_d1 != nullptr) &&
        (this->tcc_adapt_map_d2 != nullptr) &&
        (this->tcc_adapt_map_d3 != nullptr) &&
        (this->tcc_adapt_map_d4 != nullptr) &&
        (this->tcc_adapt_map_d5 != nullptr) &&
        (this->slip_rpm_target_map != nullptr);
    if (!init_tables_ok) {
        ESP_LOGE("TCC", "Some table(s) for TCC failed to load. TCC will be non functional");
    }
}

void TorqueConverter::diag_toggle_tcc_sol(bool en) {
    ESP_LOGI("TCC", "Diag request to set TCC control to %d", en);
    this->tcc_solenoid_enabled = en;
}

void TorqueConverter::fill_tcc(GearboxGear g, SensorData* sd) {
    this->map_pressure = this->get_tcc_adapt_map_pressure(g, sd);
    int end = MAX(this->map_pressure + this->min_tcc_pressure, 0);
    if (0 == this->command_p_stage) {
        // Init
        this->tcc_shift_pressure = TCC_CURRENT_SETTINGS.prefill_pressure + this->min_tcc_pressure;
        this->timer_command_p = TCC_CURRENT_SETTINGS.prefill_cycles;
        this->command_p_stage = 1;
    }
    if (1 == this->command_p_stage) {
        this->tcc_shift_pressure = TCC_CURRENT_SETTINGS.prefill_pressure + this->min_tcc_pressure;
        if (0 == this->timer_command_p) {
            this->command_p_stage = 2;
            this->timer_command_p = TCC_CURRENT_SETTINGS.prefill_cycles/2;
        }
    } else if (2 == this->command_p_stage) {
        this->tcc_shift_pressure = linear_ramp_with_timer(TCC_CURRENT_SETTINGS.prefill_pressure, end, this->timer_command_p) + this->min_tcc_pressure;
        if (0 == this->timer_command_p || this->actual_slip_abs < 10) {
            this->command_p_stage = 3;
        }
    } else {
        // Exit
        this->timer_command_p = 0;
        this->command_p_stage = 0;
        this->timer_till_adapt = 150; // Block adaptation for 3 seconds
        this->timer_till_pid = 25; // Block PID for 1/2 second
        this->current_tcc_state = InternalTccState::Slipping;
        this->tcc_shift_pressure = end;

    }

}

uint16_t TorqueConverter::calculate_slip_target(SensorData* sensors) {
    int target = SLIP_V_WHEN_OPEN;
    int inc = 0;
    if (this->pulling) {
        int pedal_as_percent = (sensors->pedal_pos*100)/250;
        target = this->slip_rpm_target_map->get_value(pedal_as_percent, sensors->input_rpm);
        targ_slip_pid = 0;
    } else {
        target = (int)interpolate_linear_array(sensors->input_rpm, 5, SLIP_X_COAST, SLIP_Z_COAST);
    }
    if (this->is_shifting) {
        if (this->upshifting) {
            if (sensors->pedal_pos >= 15  && TCC_CURRENT_SETTINGS.unlock_load_upshifts) {
                target = SLIP_V_WHEN_OPEN;
            } else if (sensors->pedal_pos < 15 && TCC_CURRENT_SETTINGS.unlock_coasting_upshifts) {
                target = SLIP_V_WHEN_OPEN;
            } else {
                target += 10; // Required
            }
        } else {
            if (sensors->pedal_pos >= 15  && TCC_CURRENT_SETTINGS.unlock_load_downshifts) {
                target = SLIP_V_WHEN_OPEN;
            } else if (sensors->pedal_pos < 15 && TCC_CURRENT_SETTINGS.unlock_coasting_downshifts) {
                target = SLIP_V_WHEN_OPEN;
            }
        }
    }

    TccReqState e_req = egs_can_hal->get_engine_tcc_override_request(100);
    if (TCC_CURRENT_SETTINGS.react_on_engine_open_request && e_req == TccReqState::Open) {
        target = SLIP_V_WHEN_OPEN;
    } else if (TCC_CURRENT_SETTINGS.react_on_engine_slip_request && e_req == TccReqState::Slipping) {
        target += 10;
    }

    if (sensors->pedal_delta_per_second >= 50 || sensors->input_rpm > 1800) {
        // TODO
        targ_slip_pid = 0;
    } else {
        if (this->actual_slip_abs - this->old_actual_slip_abs > 0 && target + inc < this->actual_slip_abs) {
            targ_slip_pid = MIN(50, (this->actual_slip_abs - target));
        }
        this->timer_inc_slip = 150;
    }
    return MAX(target, SLIP_V_WHEN_LOCKED);
}

void TorqueConverter::calculate_min_pressure(SensorData* sensors, GearboxGear current_g) {
    int min = 0;
    min = (int)interpolate_linear_array(sensors->input_rpm, 5, TCC_P_ADDER_X, TCC_P_ADDER_Z);
    if (GearboxGear::First == current_g) {
        min += 150;
    }
    this->min_tcc_pressure = min;
}

bool TorqueConverter::check_if_pulling(SensorData* sensors) {
    // Use last known value
    bool ret = this->pulling;
    if (ret) {
        bool torque_low = sensors->converted_torque <= 0;
        bool pedal_low = sensors->pedal_pos < 50;
        // Todo if cruise control is active, pedal_low is ALWAYS true
        if (torque_low && pedal_low) {
            ret = false;
            this->timer_till_adapt = 150; // 3 seconds
            this->pid_i_val = 0;
        }
    } else {
        // Pushing, check if we are pulling now
        bool torque_high = sensors->converted_torque > VEHICLE_CONFIG.engine_drag_torque / 20.0; // 1/2 drag torque
        bool pedal_high = sensors->pedal_pos > 50;
        // Todo if cruise control is active, pedal_high is ALWAYS false
        if (torque_high || pedal_high) {
            ret = true;
            this->timer_till_adapt = 150; // 3 seconds
            this->pid_i_val = 0;
            this->pid_i_val_old = 0;
        }
    }
    return ret;
}

void TorqueConverter::calculate_torque_correction(SensorData* sensors) {
    const uint8_t FILTER_SIZE = 5;
    this->filtered_engine_trq = first_order_filter(FILTER_SIZE, sensors->converted_torque*100, this->filtered_engine_trq);
    this->filtered_pump_trq = first_order_filter(FILTER_SIZE, sensors->pump_torque*100, this->filtered_pump_trq);
    if (sensors->brake_pressed && 0 == sensors->input_rpm && sensors->pedal_pos == 0) {
        if (0 == this->torque_correction_adapt) {
            this->torque_correction_adapt = (this->filtered_engine_trq - this->filtered_pump_trq);
        } else {
            this->torque_correction_adapt = first_order_filter(FILTER_SIZE, (this->filtered_engine_trq-this->filtered_pump_trq), this->torque_correction_adapt);
        }
    }
    int corr_torque = 0;
    if (false) {
        // TODO (M_CORRECTION enabled or not??)
        corr_torque = 0;
    } else {
        corr_torque = (this->torque_correction_adapt/100);
    }

    int engine_trq = sensors->converted_torque - corr_torque;

    int lambda_targ = (((int)sensors->input_rpm)*1000) / (int)(sensors->input_rpm + this->slip_target);
    int pump_trq_targ = (int)interpolate_linear_array((uint16_t)lambda_targ, 11, TCC_CFG_PTR->pump_map_x, TCC_CFG_PTR->pump_map_z);
    int x = sensors->input_rpm + this->slip_target;
    pump_trq_targ *= ((x*x) / 10000);
    pump_trq_targ /= 10000;


    int pedal_spd_abs = abs(sensors->pedal_delta_per_second);
    if (pedal_spd_abs > 50) {
        if (sensors->pedal_delta_per_second < 1) {
            float adder = 0.2 * (float)pedal_spd_abs;
            adder = MAX(adder, -50);
            engine_trq += adder;
        } else {
            float adder = 0.1 * (float)pedal_spd_abs;
            adder = MIN(50, adder);
            engine_trq += adder;
        }
    }

    if (engine_trq <= 0) {
        engine_trq += pump_trq_targ;
        if (engine_trq > 0) {
            engine_trq = 0;
        }
    } else {
        engine_trq -= pump_trq_targ;
        if (engine_trq < 0) {
            engine_trq = 0;
        }
    }
    this->tcc_engine_trq = MAX(0, engine_trq);

    int tcc_input_torque = MAX(0, this->filtered_engine_trq - (this->filtered_pump_trq/100));
    this->input_side = tcc_input_torque;
}

void TorqueConverter::process_open_or_slip_state(SensorData* sd, GearboxGear current_g) {
    this->calculate_min_pressure(sd, current_g);
    bool should_open = false;
    // Temperature processing
    if (InternalTccState::Open == this->current_tcc_state) {
        if (sd->atf_temp < -10 || sd->atf_temp > 200) {
            should_open = true;
        }
    } else {
        // Hyst. , So we don't keep transitioning between Open/Slip if temperatures are close
        if (sd->atf_temp < -15 || sd->atf_temp > 205) {
            should_open = true;
        }
    }
    if (sd->input_rpm < rpm_map_y_headers[0]) {
        should_open = true;
        this->tcc_commanded_pressure = 0; // Just in case
    }

    // Open on very low torque
    if (InternalTccState::Open == this->current_tcc_state && sd->converted_torque < VEHICLE_CONFIG.engine_drag_torque/10.0) {
        should_open = true;
    }
    // Todo friction wattage calc (unsure of relevance)

    // Now process pressures
    if (this->current_tcc_state == InternalTccState::Open && this->target_tcc_state == InternalTccState::Open) {
        // Open
        this->pid_i_val = 0;
        this->command_p_stage = 0;
        // TODO - Engagement RPM is different to slip target RPM threshold
        if (slip_target < SLIP_V_WHEN_OPEN && !should_open) {
            this->target_tcc_state = InternalTccState::Slipping;
            // Start filling!
            this->fill_tcc(current_g, sd);
        }
    } else if (this->current_tcc_state == InternalTccState::Open && this->target_tcc_state == InternalTccState::Slipping) {
        // Open -> Slipping
        if (slip_target > SLIP_V_WHEN_OPEN*1.1 || should_open) {
            this->target_tcc_state = InternalTccState::Open;
            this->tcc_commanded_pressure = 0;
        } else {
            this->fill_tcc(current_g, sd);
        }
    } else if (this->current_tcc_state == InternalTccState::Slipping && this->target_tcc_state == InternalTccState::Slipping) {
        // Slipping
        if (slip_target > SLIP_V_WHEN_OPEN*1.1 || should_open) {
            this->target_tcc_state = InternalTccState::Open;
        }
    }

    // Now process output pressures
    if (this->current_tcc_state == InternalTccState::Open && this->target_tcc_state == InternalTccState::Open) {
        // Open
        this->map_pressure = 0;
        this->pid_pressure = 0;
        this->tcc_commanded_pressure = 0;
    } else if (this->current_tcc_state == InternalTccState::Open && this->target_tcc_state == InternalTccState::Slipping) {
        // Open -> Slipping
        // PID and MAP pressure set by fill_tcc()
        this->tcc_commanded_pressure = this->tcc_shift_pressure;
    } else if (this->current_tcc_state == InternalTccState::Slipping && this->target_tcc_state == InternalTccState::Slipping) {
        // Slipping (Steady)
        this->tcc_shift_pressure = 0;
        this->map_pressure = this->get_tcc_adapt_map_pressure(current_g, sd);

        bool is_adaptable = TCC_CURRENT_SETTINGS.adapt_enable &&
            this->target_tcc_state == InternalTccState::Slipping &&
            this->current_tcc_state == InternalTccState::Slipping &&
            !is_shifting;

        // Do PID
        this->pid_i_val_old = this->pid_i_val;
        int16_t slip_delta = (int)this->actual_slip_abs - (int)this->slip_target;
        if (this->pulling) {
            this->pid_p_weight = (int)interpolate_linear_array(slip_delta, 5, TCC_PID_X, TCC_PID_PZ_PULL);
            if (0 == this->timer_till_pid) {
                    this->pid_i_weight = (int)interpolate_linear_array(slip_delta, 5, TCC_PID_X, TCC_PID_IZ_PULL);
            } else {
                this->pid_i_val = ((this->pid_p_weight * slip_delta) / 1000) * -10;
            }

        } else {
            pid_p_weight = (int)interpolate_linear_array(slip_delta, 5, TCC_PID_X, TCC_PID_PZ_PUSH);
            pid_i_weight = (int)interpolate_linear_array(slip_delta, 5, TCC_PID_X, TCC_PID_IZ_PUSH);
        }
        int i_adder = (((this->pid_i_weight * slip_delta) / 100) * 20) / 100;
        this->pid_i_val += i_adder;
        // Dynamic PID modifier
        if (this->actual_slip_abs < SLIP_V_WHEN_OPEN) {
            if (sd->pedal_delta_per_second > 20) {
                this->pid_i_val = 0;
            }
        }

        int p = (this->pid_p_weight * slip_delta) / 1000;
        this->pid_pressure = p + (this->pid_i_val/10);
        this->tcc_commanded_pressure = this->pid_pressure + this->min_tcc_pressure + this->map_pressure;

        if (this->tcc_commanded_pressure > 15000 || this->tcc_commanded_pressure < 0) {
            is_adaptable = false;
            this->pid_i_val = this->pid_i_val_old;
            this->tcc_commanded_pressure = MIN(15000, MAX(0, this->tcc_commanded_pressure));
        }

        if (0 == this->timer_till_adapt && is_adaptable) {
            // Write the adapt value
            if (abs(sd->pedal_delta_per_second) > 50 || sd->input_rpm > 3500) {
                this->timer_till_adapt = 150;
            } else {
                StoredMap* map = this->get_tcc_adapt_map(current_g);
                if (nullptr != map) {
                    // Correction pressure to reduce reliance on PID
                    int new_value = MAX(0, this->tcc_commanded_pressure - this->min_tcc_pressure);
                    int x_idx = 0;
                    int y_idx = 0;
                    for (int i = 1; i < 6; i++) {
                        if (TCC_ADAPT_MAP_X[i] > engine_load_percent) {
                            break;
                        }
                        x_idx = i;
                    }
                    for (int i = 1; i < 6; i++) {
                        if (TCC_ADAPT_MAP_Y[i] > sd->atf_temp) {
                            break;
                        }
                        y_idx = i;
                    }
                    int old_v = map->get_value(engine_load_percent, sd->atf_temp);
                    map->add_value(new_value, engine_load_percent, sd->atf_temp, 10.0);
                    // Update PID I value
                    this->pid_i_val -= (new_value - old_v);
                    if (new_value > old_v) {
                        // Increase upper cells
                        for (int x = x_idx; x < 6; x++) {
                            for (int y = y_idx; y < 6; y++) {
                                old_v = map->get_value(TCC_ADAPT_MAP_X[x], TCC_ADAPT_MAP_Y[y]);
                                map->add_value(MAX(new_value, old_v), TCC_ADAPT_MAP_X[x], TCC_ADAPT_MAP_Y[y], 10.0);
                            }
                        }
                    } else if (new_value < old_v) {
                        // Decrease lower cells
                        for (int x = 0; x < x_idx; x++) {
                            for (int y = 0; y < y_idx; y++) {
                                old_v = map->get_value(TCC_ADAPT_MAP_X[x], TCC_ADAPT_MAP_Y[y]);
                                map->add_value(MIN(new_value, old_v), TCC_ADAPT_MAP_X[x], TCC_ADAPT_MAP_Y[y], 10.0);
                            }
                        }

                    }
                }
                this->timer_till_adapt = 25; // 500ms till the next write is allowed
            }
        }
    } else {
        // Slipping -> Open
        this->pid_pressure = 0;
        this->map_pressure = 0;
        if (this->tcc_commanded_pressure > 100) {
            this->tcc_commanded_pressure -= 100;
        } else {
            this->tcc_commanded_pressure = 0;
            this->current_tcc_state = InternalTccState::Open;
        }
    }

}

void TorqueConverter::update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors) {
    // Timers
    if (this->timer_inc_slip > 0) {
        this->timer_inc_slip -= 1;
    }
    if (this->timer_till_adapt > 0) {
        this->timer_till_adapt -= 1;
    }
    if (this->timer_command_p > 0) {
        this->timer_command_p -= 1;
    }
    if (this->timer_till_pid > 0) {
        this->timer_till_pid -= 1;
    }
    // Pulling / Pushing check
    this->pulling = this->check_if_pulling(sensors);

    if (curr_gear != targ_gear) {
        this->timer_till_adapt = MAX(this->timer_till_adapt, 50); // Wait 1 second after shifting to adapt (Also don't adapt during shifts)
    }

    this->calculate_torque_correction(sensors);
    int load_as_percent = abs(((int)this->tcc_engine_trq*100) / this->rated_max_torque);
    this->engine_load_percent = load_as_percent;

    this->filtered_engine_rpm = first_order_filter(3, (int)sensors->engine_rpm*100, this->filtered_engine_rpm);
    this->filtered_input_rpm = first_order_filter(3, (int)sensors->input_rpm*100, this->filtered_input_rpm);
    this->old_actual_slip_abs = this->actual_slip_abs;
    this->actual_slip_abs = abs(this->filtered_engine_rpm - this->filtered_input_rpm) / 100;

    // Conditions for no TCC
    if (
        !this->tcc_solenoid_enabled || // Diagnostic request
        !init_tables_ok || // Some data was not initialized or invalid
        sol_tcc->is_disabled() // ISR of the TCC solenoid is disabled
    ) {
        this->tcc_commanded_pressure = 0;
        this->pid_pressure = 0;
        this->map_pressure = 0;
        this->tcc_shift_pressure = 0;
        pm->set_target_tcc_pressure(this->tcc_commanded_pressure);
        this->current_tcc_state = InternalTccState::Open;
        this->target_tcc_state = InternalTccState::Open;
        this->slip_target = SLIP_V_WHEN_OPEN*2; // 200RPM = Way out of open range
        this->timer_command_p = 0;
        this->timer_till_adapt = 10;
        this->timer_till_pid = 10;
        return;
    }

    GearboxGear cmp_gear = curr_gear;
    this->slip_target = this->calculate_slip_target(sensors);
    // TCC Disable check
    if (
        ((cmp_gear == GearboxGear::First && !TCC_CURRENT_SETTINGS.enable_d1)||
        (cmp_gear == GearboxGear::Second && !TCC_CURRENT_SETTINGS.enable_d2)||
        (cmp_gear == GearboxGear::Third && !TCC_CURRENT_SETTINGS.enable_d3) ||
        (cmp_gear == GearboxGear::Fourth && !TCC_CURRENT_SETTINGS.enable_d4)||
        (cmp_gear == GearboxGear::Fifth && !TCC_CURRENT_SETTINGS.enable_d5))
    ) {
        this->slip_target = SLIP_V_WHEN_OPEN*2; // 200RPM = Way out of open range
    }
    this->process_open_or_slip_state(sensors, curr_gear);

    if (is_shifting) {
        this->timer_till_pid = 50;
        this->timer_till_adapt = 150;
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
        ret = TccClutchStatus::OpenToSlipping;
    }
    return ret;
}

void TorqueConverter::set_stationary() {
    this->was_stationary = true;
}


uint8_t TorqueConverter::get_current_state() {
    return (uint8_t)this->current_tcc_state;
}

uint8_t TorqueConverter::get_target_state() {
    return (uint8_t)this->target_tcc_state;
}

uint16_t TorqueConverter::get_slip_now() {
    return this->actual_slip_abs;
}

uint16_t TorqueConverter::get_command_pressure() {
    return this->tcc_commanded_pressure;
}

uint16_t TorqueConverter::get_map_pressure() {
    return this->map_pressure;
}

int16_t TorqueConverter::get_pid_pressure() {
    return this->pid_pressure;
}

uint8_t TorqueConverter::get_adapt_timer() {
    return this->timer_till_adapt;
}

uint8_t TorqueConverter::get_pid_timer() {
    return this->timer_till_pid;
}

void TorqueConverter::shift_start(bool upshift, bool release_shifting) {
    this->is_shifting = true;
    this->was_shifting = true;
    this->release_shifting = release_shifting;
    this->upshifting = upshift;
}
void TorqueConverter::shift_end() {
    this->is_shifting = false;
    this->upshifting = false;
    this->release_shifting = false;
}
