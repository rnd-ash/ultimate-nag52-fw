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

TorqueConverter::TorqueConverter(uint16_t max_gb_rating)  {
    tcc_learn_lockup_map = new StoredTcuMap("TCC_LOCK", 5, tcc_learn_x_headers, tcc_learn_y_headers, 5, 1, tcc_learn_default_data);
    if (this->tcc_learn_lockup_map->init_status() != ESP_OK) {
        delete[] this->tcc_learn_lockup_map;
    }
    int16_t* data = this->tcc_learn_lockup_map->get_current_data();
    for (int i = 0; i < 5; i++) {
        ESP_LOGI("TCC", "Adapt value for gear %d - %d mBar", i+1, data[i]);
    }
    // range of adaptation is 1/10 - 1/3 of the rating of the gearbox
    // W5A330 - 33Nm - 110Nm
    // W5A580 - 58Nm - 193Nm
    this->high_torque_adapt_limit = max_gb_rating / 3;
    this->low_torque_adapt_limit = max_gb_rating / 10;
}


void TorqueConverter::update(GearboxGear curr_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting) {
    if (is_shifting) { // Reset strikes when changing gears!
        this->strike_count = 0;
        this->adapt_lock_count = 0;
        last_adj_time = sensors->current_timestamp_ms;
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
    } else {
        // We can lock now!
        if (this->was_idle) {
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
        } else {
            // We are just driving, TCC is free to lockup
            if (!initial_ramp_done) {
#define TCC_FAST_RAMP_STEP 10 // ~= 200mBar/sec
                // We are in stage of ramping TCC pressure up to initial lock phase as learned by TCC
                int delta = MIN(TCC_FAST_RAMP_STEP, abs((int)this->curr_tcc_target - (int)this->base_tcc_pressure));
                if (delta > TCC_FAST_RAMP_STEP) {
                    if (this->curr_tcc_target > this->base_tcc_pressure) {
                        this->base_tcc_pressure += TCC_FAST_RAMP_STEP;
                    } else if (this->curr_tcc_target < this->base_tcc_pressure) {
                        this->base_tcc_pressure -= TCC_FAST_RAMP_STEP;
                    }
                } else {
                    this->base_tcc_pressure = this->curr_tcc_target;
                    initial_ramp_done = true;
                }
                this->curr_tcc_pressure = this->base_tcc_pressure;
                last_adj_time = sensors->current_timestamp_ms;
            } else if (sensors->current_timestamp_ms - last_adj_time > 500) { // Allowed to adjust
                last_adj_time = sensors->current_timestamp_ms;
                bool learning = false;
                // Learning phase / dynamic phase
                if (!is_shifting && this->tcc_learn_lockup_map != nullptr) {
                    // Learning phase check
                    if (sensors->static_torque >= this->low_torque_adapt_limit && sensors->static_torque <= this->high_torque_adapt_limit) {
                        if (sensors->tcc_slip_rpm > 0 && sensors->tcc_slip_rpm < TCC_ADAPT_CONSIDERED_LOCK) {
                            adapt_lock_count++;
                        } else {
                            learning = true;
                            this->base_tcc_pressure += 25;
                        }
                    } else {
                        adapt_lock_count = 0;
                    }
                    if (adapt_lock_count == 100) { // ~= 2 seconds
                        // Modify map
                        int16_t* modify = this->tcc_learn_lockup_map->get_current_data();
                        int16_t curr_v = modify[(uint8_t)(curr_gear)-1];
                        if (abs(this->base_tcc_pressure-curr_v) > 100) {
                            ESP_LOGI("TCC", "Adjusting TCC adaptation for gear %d. Was %d mBar, now %d mBar", (uint8_t)curr_gear, curr_v, this->base_tcc_pressure);
                            modify[(uint8_t)(curr_gear)-1] = (int16_t)this->base_tcc_pressure;
                        }
                    }
                }
                this->curr_tcc_pressure = this->base_tcc_pressure;
                // Dynamic TCC pressure increase based on torque
                if (!learning) {
                    if (sensors->static_torque > high_torque_adapt_limit) {
                        int torque_delta = sensors->static_torque - high_torque_adapt_limit;
                        this->curr_tcc_pressure = this->base_tcc_pressure + (6*torque_delta); // 5mBar per Nm
                    } else if (sensors->static_torque < 0) {
                        if (this->curr_tcc_pressure > TCC_PREFILL) {
                            this->curr_tcc_pressure = this->base_tcc_pressure - 100;
                        }
                    }
                }
                bool adj = false;
                if (sensors->static_torque < 0 && sensors->tcc_slip_rpm < -200) {
                    this->base_tcc_pressure += 5;
                    adj = true;
                } else if (sensors->static_torque < 0 && sensors->tcc_slip_rpm < -50) {
                    this->base_tcc_pressure -= 5;
                    adj = true;
                }
                if (adj) {
                    // Too much slip
                    int16_t* modify = this->tcc_learn_lockup_map->get_current_data();
                    int16_t curr_v = modify[(uint8_t)(curr_gear)-1];
                    if (abs(this->base_tcc_pressure-curr_v) > 100) {
                        ESP_LOGI("TCC", "Adjusting TCC adaptation for gear %d. Was %d mBar, now %d mBar", (uint8_t)curr_gear, curr_v, this->base_tcc_pressure);
                        modify[(uint8_t)(curr_gear)-1] = (int16_t)this->base_tcc_pressure;
                    }
                }
            }
        }
    }


    if (this->curr_tcc_pressure > 2000) {
        this->curr_tcc_pressure = 2000;
    }
    pm->set_target_tcc_pressure(this->curr_tcc_pressure);

/*
    int trq = sensors->static_torque;
    if (trq < 0) {
        trq *= -1;
    }
    // Only think about lockup on positive torque
    int max_allowed_slip;
    int min_allowed_slip;
    float temp_time_multiplier = pm->get_tcc_temp_multiplier(sensors->atf_temp);
    if (profile == nullptr) {
        max_allowed_slip = MAX(100, trq);
        min_allowed_slip = MAX(10, trq/2);
    } else {
        TccLockupBounds bounds = profile->get_tcc_lockup_bounds(sensors, curr_gear);
        max_allowed_slip = bounds.max_slip_rpm;
        min_allowed_slip = bounds.min_slip_rpm;
        // When producing less thats 44Nm, we should always be locked
        // as that is the turbines inertia force, if we are not then we trigger
        // more pressure with this code 
        if (sensors->static_torque <= 44 && sensors->static_torque > 0) {
            max_allowed_slip = 10;
            min_allowed_slip = 0;
        }
        // This means that at 1000Rpm, we will get 0.9 the slip (More slip)
        // At 2100RPM, we will get half the slip as at 1100rpm
        // Dynamic slip :D
        float multiplier = MIN(0.9, (float)(1000.0/MAX(1000.0, sensors->engine_rpm)));
        max_allowed_slip *= multiplier;
        min_allowed_slip *= multiplier;
    }
    int slip = abs(sensors->tcc_slip_rpm);
    float rpm_multi = 2000.0/sensors->engine_rpm;
    if (is_shifting || sensors->current_timestamp_ms-sensors->last_shift_time < 1500) {
        this->last_inc_time = sensors->current_timestamp_ms;
        strike_count = 0;
        goto write_pressure;
    }
    if (slip > max_allowed_slip && strike_count < 20) {
        strike_count++;
    } else if (slip < max_allowed_slip) {
        strike_count = 0;
    }
    // Quick activate of slip mode
    if (sensors->input_rpm > 1000 && this->curr_tcc_pressure < 1100) {
        this->curr_tcc_pressure = 1100;
        goto write_pressure;
    }
    if (sensors->current_timestamp_ms - last_inc_time > 500 * rpm_multi * temp_time_multiplier && strike_count >= 10) {
        if (slip > max_allowed_slip) {
            if (this->curr_tcc_pressure < 1100) {
                this->curr_tcc_pressure += 50; // 50mbar
            } else if (this->curr_tcc_pressure >= 1200) {
                this->curr_tcc_pressure += 10; // 50mbar
            } else {
                this->curr_tcc_pressure += 25; // 50mbar
            }
            this->last_inc_time = sensors->current_timestamp_ms;
        } else if (sensors->tcc_slip_rpm < min_allowed_slip && this->curr_tcc_pressure >= 1100 && sensors->static_torque > 40) {
            // Decrease pressure, but only if we have pedal input
            this->curr_tcc_pressure -= 50; // 50mBar
            this->last_inc_time = sensors->current_timestamp_ms;
        }
    }
write_pressure:
    if (this->curr_tcc_pressure <= 0) { // Just to be safe!
        this->curr_tcc_pressure = 0;
    }
    this->state = (slip > 200 || is_shifting) ? ClutchStatus::Slipping : ClutchStatus::Closed;
    pm->set_target_tcc_pressure((uint16_t)(this->curr_tcc_pressure));
*/
}

void TorqueConverter::on_shift_complete(uint64_t now) {

}

// 1600 - lock
// 1000 - slip
// 500 - prefill
// void TorqueConverter::on_shift_start(uint64_t now, bool is_downshift, SensorData* sensors, float shift_firmness) {
//     if (this->curr_tcc_pressure < 1200) {
//         this->curr_tcc_pressure = 1200;
//     } else if (sensors->static_torque > 0 && abs(sensors->tcc_slip_rpm) < 20) {
//         uint32_t additional_reduction = scale_number(shift_firmness*10, 100, 0, 0, 100);
//         this->curr_tcc_pressure -= additional_reduction;
//     }
// }

ClutchStatus TorqueConverter::get_clutch_state(void) {
    return this->state;
}