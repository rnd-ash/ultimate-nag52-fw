#include "torque_converter.h"
#include "solenoids/solenoids.h"
#include "tcu_maths.h"

// 1400 mBar ~= locking (C200CDI)
// 1700 mBar ~= locking (E55 AMG)

static const uint16_t TCC_ADAPT_CONSIDERED_LOCK = 25; 
static const uint16_t TCC_PREFILL = 500u; // mBar
const int16_t tcc_learn_x_headers[5] = {1,2,3,4,5};
const int16_t tcc_learn_y_headers[1] = {1};
const int16_t tcc_learn_default_data[5] = {900, 900, 900, 900, 900};
const uint16_t TCC_MIN_LOCKING_RPM = 1100;

static const uint16_t TCC_ADJ_INTERVAL_MS = 500;

TorqueConverter::TorqueConverter(uint16_t max_gb_rating)  {
    tcc_learn_lockup_map = new StoredTcuMap("TCC_LOCK", 5, tcc_learn_x_headers, tcc_learn_y_headers, 5, 1, tcc_learn_default_data);
    if (this->tcc_learn_lockup_map->init_status() != ESP_OK) {
        delete[] this->tcc_learn_lockup_map;
    }
    int16_t* data = this->tcc_learn_lockup_map->get_current_data();
    for (int i = 0; i < 5; i++) {
        //data[i] = tcc_learn_default_data[i];
        ESP_LOGI("TCC", "Adapt value for gear %d - %d mBar", i+1, data[i]);
    }
    // range of adaptation is 1/10 - 1/3 of the rating of the gearbox
    // W5A330 - 33Nm - 55Nm
    // W5A580 - 58Nm - 96Nm
    this->high_torque_adapt_limit = max_gb_rating / 6;
    this->low_torque_adapt_limit = max_gb_rating / 10;
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

void TorqueConverter::update(GearboxGear curr_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting) {
    if (is_shifting) { // Reset strikes when changing gears!
        this->strike_count = 0;
        this->adapt_lock_count = 0;
        last_adj_time = sensors->current_timestamp_ms;
        this->reset_rpm_samples(sensors);
        this->was_shifting = true;
    } else {
        this->tmp_lookup_gear = 0xFF;
    }
    if (sensors->input_rpm <= TCC_MIN_LOCKING_RPM) { // RPM too low!
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

        if (sensors->engine_rpm > 900) {
            this->base_tcc_pressure = TCC_PREFILL;
        } else {
            this->base_tcc_pressure = 0;
        }
        this->curr_tcc_pressure = this->base_tcc_pressure;
        this->reset_rpm_samples(sensors);
    } else {
        // We can lock now!
        if (this->was_idle || (was_shifting && !is_shifting)) {
            was_shifting = false;
            // We were too low, but now we can lock! (RPM increasing)
            this->was_idle = false;
            if (this->tcc_learn_lockup_map != nullptr) {
                this->curr_tcc_target = this->tcc_learn_lockup_map->get_value((float)curr_gear, 1.0);
                ESP_LOGI("TCC", "Learn cell value is %d mBar", curr_tcc_target);
                this->initial_ramp_done = false;
            } else {
                this->initial_ramp_done = true;
            }
            this->base_tcc_pressure = TCC_PREFILL;
            this->curr_tcc_pressure = TCC_PREFILL;
            last_adj_time = sensors->current_timestamp_ms;
            this->reset_rpm_samples(sensors);
        } else {
            // We are just driving, TCC is free to lockup
            if (!initial_ramp_done) {
#define TCC_FAST_RAMP_STEP 5 // ~= 200mBar/sec
                // We are in stage of ramping TCC pressure up to initial lock phase as learned by TCC
                int delta = MIN(TCC_FAST_RAMP_STEP, abs((int)this->curr_tcc_target - (int)this->base_tcc_pressure));
                if (abs(delta) > TCC_FAST_RAMP_STEP) {
                    if (this->curr_tcc_target > this->base_tcc_pressure) {
                        this->base_tcc_pressure += delta;
                    } else if (this->curr_tcc_target < this->base_tcc_pressure) {
                        this->base_tcc_pressure -= delta;
                    }
                } else {
                    this->base_tcc_pressure = this->curr_tcc_target;
                    initial_ramp_done = true;
                }
                this->curr_tcc_pressure = this->base_tcc_pressure;
                last_adj_time = sensors->current_timestamp_ms;
                this->reset_rpm_samples(sensors);
            }
            if (initial_ramp_done) {
                bool learning = false;
                if (sensors->current_timestamp_ms - last_adj_time > TCC_ADJ_INTERVAL_MS) { // Allowed to adjust
                    last_adj_time = sensors->current_timestamp_ms;
                    // Learning phase / dynamic phase
                    if (!is_shifting && this->tcc_learn_lockup_map != nullptr) {
                        // Learning phase check
                        // Requires:
                        // * torque in bounds
                        // * Engine RPM - Less than TCC stall speed
                        if (sensors->static_torque >= this->low_torque_adapt_limit && sensors->static_torque <= this->high_torque_adapt_limit && sensors->engine_rpm < 2500) {
                            if (sensors->tcc_slip_rpm > 0 && sensors->tcc_slip_rpm < TCC_ADAPT_CONSIDERED_LOCK) {
                                adapt_lock_count++;
                            } else {
                                learning = true;
                                this->base_tcc_pressure += 10;
                            }
                        } else {
                            adapt_lock_count = 0;
                        }
                        if (adapt_lock_count == 2000/TCC_ADJ_INTERVAL_MS) { // ~= 2 seconds
                            this->adjust_map_cell(curr_gear, this->base_tcc_pressure);
                        }
                    }
                    this->curr_tcc_pressure = this->base_tcc_pressure;
                    bool adj = false;
                    if (sensors->static_torque < 0 && abs(sensors->tcc_slip_rpm) > 100 && sensors->pedal_pos == 0) {
                        this->base_tcc_pressure += 10;
                        adj = true;
                    }// else if (sensors->static_torque < 0 && abs(sensors->tcc_slip_rpm) < 20) {
                    //    this->base_tcc_pressure -= 10;
                    //    adj = true;
                    //}
                    if (adj) {
                        this->adjust_map_cell(curr_gear, this->base_tcc_pressure);
                    }
                }
                // Dynamic TCC pressure increase based on torque
                this->curr_tcc_pressure = this->base_tcc_pressure;
                if (!learning) {
                    if (sensors->static_torque > high_torque_adapt_limit) {
                        int torque_delta = sensors->static_torque - high_torque_adapt_limit;
                        this->curr_tcc_pressure += (1.4*torque_delta); // 2mBar per Nm
                    } else if (sensors->static_torque < 0) {
                        if (this->curr_tcc_pressure > TCC_PREFILL) {
                            this->curr_tcc_pressure -= 50;
                        }
                    }
                }
                if (sensors->output_rpm > 1500 && this->curr_tcc_pressure > TCC_PREFILL) {
                    this->curr_tcc_pressure = (uint32_t)(float)this->curr_tcc_pressure * ((float)scale_number(sensors->output_rpm, 100, 125, 1500, 2500)/100.0);
                }
            }
        }
    }
    if (this->base_tcc_pressure > 1800) {
        this->base_tcc_pressure = 1800;
    }
    if (this->curr_tcc_pressure > 7000) {
        this->curr_tcc_pressure = 7000;
    }
    pm->set_target_tcc_pressure(this->curr_tcc_pressure);
}

ClutchStatus TorqueConverter::get_clutch_state(void) {
    return this->state;
}

void TorqueConverter::on_shift_start(int targ_g) {
    this->curr_tcc_target = targ_g;
}