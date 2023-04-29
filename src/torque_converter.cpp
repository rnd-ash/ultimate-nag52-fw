#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcu_maths.h"
#include "nvs/eeprom_impl.h"

// 1400 mBar ~= locking (C200CDI)
// 1700 mBar ~= locking (E55 AMG)

const int16_t tcc_learn_x_headers[5] = {1,2,3,4,5};
const int16_t tcc_learn_y_headers[1] = {1};
const int16_t tcc_learn_default_data[5] = {900, 900, 900, 900, 900};

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
    if (curr_gear != targ_gear && is_shifting) {
        // Check for when we should not be powering TCC
        cmp_gear = targ_gear;
    }
    if (
        (cmp_gear == GearboxGear::First && !TCC_CURRENT_SETTINGS.enable_d1) ||
        (cmp_gear == GearboxGear::Second && !TCC_CURRENT_SETTINGS.enable_d2) ||
        (cmp_gear == GearboxGear::Third && !TCC_CURRENT_SETTINGS.enable_d3) ||
        (cmp_gear == GearboxGear::Fourth && !TCC_CURRENT_SETTINGS.enable_d4) ||
        (cmp_gear == GearboxGear::Fifth && !TCC_CURRENT_SETTINGS.enable_d5)
    ) {
        // Not enabled in this gear. No need to do any calculations
        this->was_idle = true;
        this->last_idle_timestamp = 0;
        this->curr_tcc_pressure = 0;
        this->curr_tcc_target = 0;
        this->base_tcc_pressure = 0;
        this->initial_ramp_done = false;
        this->strike_count = 0;
        this->adapt_lock_count = 0;
        this->state = ClutchStatus::Open;
    } else {
        uint32_t input_rpm = sensors->input_rpm;
        int trq = MAX(sensors->static_torque, sensors->driver_requested_torque);
        if (input_rpm <= TCC_CURRENT_SETTINGS.min_locking_rpm) {
            if (this->last_idle_timestamp == 0) {
                this->last_idle_timestamp = sensors->current_timestamp_ms;
            }
        } else {
            this->last_idle_timestamp = 0;
        }
        if (input_rpm <= TCC_CURRENT_SETTINGS.min_locking_rpm && sensors->current_timestamp_ms - this->last_idle_timestamp > 100) { // RPM too low (More than 100ms under the RPM target)!
            last_adj_time = sensors->current_timestamp_ms;
            if (!this->was_idle) {
                // Input has fallen below locking RPM (Slowing down). Do our once-actions here
                this->was_idle = true;
                this->initial_ramp_done = false;
                this->strike_count = 0;
                this->adapt_lock_count = 0;
                this->state = ClutchStatus::Open;
            } else {
                // Input is still lower than locking RPM
            }
            if (sensors->engine_rpm > TCC_CURRENT_SETTINGS.prefill_min_engine_rpm) {
                this->base_tcc_pressure = TCC_CURRENT_SETTINGS.prefill_pressure;
            } else {
                this->base_tcc_pressure = 0;
            }
            this->curr_tcc_pressure = this->base_tcc_pressure;
            this->reset_rpm_samples(sensors);
        } else {
            // We can lock now!
            if (this->was_idle) {
                was_shifting = false;
                // We were too low, but now we can lock! (RPM increasing)
                this->was_idle = false;
                if (this->tcc_learn_lockup_map != nullptr) {
                    this->curr_tcc_target = this->tcc_learn_lockup_map->get_value((float)cmp_gear, 1.0);
                    ESP_LOGI("TCC", "Learn cell value is %lu mBar", curr_tcc_target);
                    this->initial_ramp_done = false;
                    this->base_tcc_pressure = MAX(0, this->curr_tcc_target-TCC_CURRENT_SETTINGS.base_pressure_offset_start_ramp);
                    this->curr_tcc_pressure = MAX(0, this->curr_tcc_target-TCC_CURRENT_SETTINGS.base_pressure_offset_start_ramp);
                } else {
                    this->initial_ramp_done = true;
                    this->base_tcc_pressure = TCC_CURRENT_SETTINGS.prefill_pressure;
                    this->curr_tcc_pressure = TCC_CURRENT_SETTINGS.prefill_pressure;
                }
                last_adj_time = sensors->current_timestamp_ms;
                this->reset_rpm_samples(sensors);
            } else {
                // We are just driving, TCC is free to lockup
                if (!initial_ramp_done) {
                    if (is_shifting) {
                        this->base_tcc_pressure = MAX(this->base_tcc_pressure, this->curr_tcc_target-100);
                    } else {
                        // We are in stage of ramping TCC pressure up to initial lock phase as learned by TCC
                        float ramp = scale_number(abs(sensors->tcc_slip_rpm), &TCC_CURRENT_SETTINGS.pressure_increase_ramp_settings);
                        int delta = MIN(ramp+1, this->base_tcc_pressure - this->curr_tcc_target);
                        if (delta > ramp) {
                            this->base_tcc_pressure += delta;
                        } else {
                            this->base_tcc_pressure = this->curr_tcc_target;
                            initial_ramp_done = true;
                        }
                    }
                    this->curr_tcc_pressure = this->base_tcc_pressure;
                    last_adj_time = sensors->current_timestamp_ms;
                    this->reset_rpm_samples(sensors);
                } else {
                    bool learning = false;
                    if (TCC_CURRENT_SETTINGS.adapt_enable && sensors->current_timestamp_ms - last_adj_time > TCC_ADJ_INTERVAL_MS) { // Allowed to adjust
                        last_adj_time = sensors->current_timestamp_ms;
                        // Learning phase / dynamic phase
                        if (!is_shifting && this->tcc_learn_lockup_map != nullptr) {
                            // Learning phase check
                            // Requires:
                            // * torque in bounds
                            // * Engine RPM - Less than TCC stall speed
                            if (trq >= TCC_CURRENT_SETTINGS.min_torque_adapt && trq <= TCC_CURRENT_SETTINGS.max_torque_adapt && sensors->engine_rpm < TCC_CURRENT_SETTINGS.tcc_stall_speed) {
                                if (sensors->tcc_slip_rpm > 0 && sensors->tcc_slip_rpm < TCC_CURRENT_SETTINGS.lock_rpm_threshold) {
                                    adapt_lock_count++;
                                } else {
                                    learning = true;
                                    this->base_tcc_pressure += TCC_CURRENT_SETTINGS.adapt_pressure_inc;
                                }
                            } else {
                                adapt_lock_count = 0;
                            }
                            if (adapt_lock_count == TCC_CURRENT_SETTINGS.adapt_lock_detect_time/TCC_ADJ_INTERVAL_MS) {
                                this->adjust_map_cell(cmp_gear, this->base_tcc_pressure);
                            }
                        }
                        this->curr_tcc_pressure = this->base_tcc_pressure;
                        bool adj = false;
                        if (sensors->static_torque < 0 && abs(sensors->tcc_slip_rpm) > TCC_CURRENT_SETTINGS.pulling_slip_rpm_high_threhold && sensors->pedal_pos == 0) {
                            this->base_tcc_pressure += TCC_CURRENT_SETTINGS.adapt_pressure_inc;
                            adj = true;
                        } else if (sensors->static_torque < 0 && abs(sensors->tcc_slip_rpm) < TCC_CURRENT_SETTINGS.pulling_slip_rpm_low_threshold) {
                            this->base_tcc_pressure -= TCC_CURRENT_SETTINGS.adapt_pressure_inc;
                            adj = true;
                        }
                        if (adj) {
                            this->adjust_map_cell(cmp_gear, this->base_tcc_pressure);
                        }
                    }
                    // Dynamic TCC pressure increase based on torque
                    this->curr_tcc_pressure = this->base_tcc_pressure;
                    if (trq > TCC_CURRENT_SETTINGS.max_torque_adapt) {
                        int torque_delta = trq - TCC_CURRENT_SETTINGS.max_torque_adapt;
                        this->curr_tcc_pressure += (TCC_CURRENT_SETTINGS.reaction_torque_multiplier*torque_delta);
                    } else if (sensors->static_torque < TCC_CURRENT_SETTINGS.min_torque_adapt) {
                        if (this->curr_tcc_pressure > TCC_CURRENT_SETTINGS.prefill_pressure) {
                            this->curr_tcc_pressure -= scale_number(sensors->static_torque, &TCC_CURRENT_SETTINGS.load_dampening);
                        }
                    }
                    if (sensors->output_rpm > TCC_CURRENT_SETTINGS.pressure_multiplier_output_rpm.raw_min && this->curr_tcc_pressure > TCC_CURRENT_SETTINGS.prefill_pressure) {
                        this->curr_tcc_pressure = (uint32_t)(float)this->curr_tcc_pressure * scale_number(sensors->output_rpm, &TCC_CURRENT_SETTINGS.pressure_multiplier_output_rpm);
                    }
                }
            }
        }
    }
    if (this->base_tcc_pressure > TCC_CURRENT_SETTINGS.max_allowed_bite_pressure) {
        this->base_tcc_pressure = TCC_CURRENT_SETTINGS.max_allowed_bite_pressure;
    }
    if (this->curr_tcc_pressure > TCC_CURRENT_SETTINGS.max_allowed_pressure_longterm) {
        this->curr_tcc_pressure = TCC_CURRENT_SETTINGS.max_allowed_pressure_longterm;
    }
    pm->set_target_tcc_pressure(this->curr_tcc_pressure);
}

ClutchStatus TorqueConverter::get_clutch_state(void) {
    return this->state;
}