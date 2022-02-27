#include "gearbox.h"
#include "scn.h"
#include "nvs/eeprom_config.h"
#include "adv_opts.h"

const float diff_ratio_f = (float)DIFF_RATIO / 1000.0;

#define max(a,b) \
  ({ \
    typeof (a) _a = (a); \
    typeof (b) _b = (b); \
    _a > _b ? _a : _b; \
  })

#define CLAMP(value, min, max) \
    if (value < min) { \
        value = min; \
    } else if (value >= max) { \
        value = max-1; \
    } \

// ONLY FOR FORWARD GEARS!
int calc_input_rpm_from_req_gear(int output_rpm, GearboxGear req_gear) {
    switch (req_gear) {
        case GearboxGear::First:
            return output_rpm*RAT_1;
        case GearboxGear::Second:
            return output_rpm*RAT_2;
        case GearboxGear::Third:
            return output_rpm*RAT_3;
        case GearboxGear::Fourth:
            return output_rpm*RAT_4;
        case GearboxGear::Fifth:
            return output_rpm*RAT_5;
        default:
            return 0;
    }
}

Gearbox::Gearbox() {
    this->current_profile = nullptr;
    egs_can_hal->set_drive_profile(GearboxProfile::Underscore); // Uninitialized
    this->profile_mutex = portMUX_INITIALIZER_UNLOCKED;
    this->sensor_data = SensorData {
        .input_rpm = 0,
        .engine_rpm = 0,
        .output_rpm = 0,
        .voltage = 12000,   
        .pedal_pos = 0,
        .atf_temp = 0,
        .static_torque = 0,
        .max_torque = 0,
        .min_torque = 0,
        .tcc_slip_rpm = 0,
        .last_shift_time = 0,
        .current_timestamp_ms = (uint64_t)(esp_timer_get_time()/1000),
        .is_braking = false,
        .d_output_rpm = 0
    };
    this->tcc = new TorqueConverter();
    this->pressure_mgr = new PressureManager(&this->sensor_data);
}

void Gearbox::set_profile(AbstractProfile* prof) {
    if (prof != nullptr) { // Only change if not nullptr!
        portENTER_CRITICAL(&this->profile_mutex);
        this->current_profile = prof;
        // Set CAN display
        egs_can_hal->set_drive_profile(this->current_profile->get_profile());
        portEXIT_CRITICAL(&this->profile_mutex);
    }
}

bool Gearbox::start_controller() {
    xTaskCreatePinnedToCore(Gearbox::start_controller_internal, "GEARBOX", 32768, (void*)this, 10, nullptr, 1);
    return true;
}

GearboxGear gear_from_idx(uint8_t idx) {
    switch (idx) {
        case 1:
            return GearboxGear::First;
        case 2:
            return GearboxGear::Second;
        case 3:
            return GearboxGear::Third;
        case 4:
            return GearboxGear::Fourth;
        case 5:
            return GearboxGear::Fifth;
        case 6:
            return GearboxGear::Reverse_First;
        case 7:
            return GearboxGear::Reverse_Second;
        default:
            return GearboxGear::SignalNotAvaliable;
    }
}

bool is_controllable_gear(GearboxGear g) {
    switch (g) {
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
        case GearboxGear::First:
        case GearboxGear::Second:
        case GearboxGear::Third:
        case GearboxGear::Fourth:
        case GearboxGear::Fifth:
            return true;
        case GearboxGear::Park:
        case GearboxGear::SignalNotAvaliable:
        case GearboxGear::Neutral:
        default:
            return false;
    }
}

bool is_fwd_gear(GearboxGear g) {
    switch (g) {
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return false;
        case GearboxGear::First:
        case GearboxGear::Second:
        case GearboxGear::Third:
        case GearboxGear::Fourth:
        case GearboxGear::Fifth:
            return true;
        case GearboxGear::Park:
        case GearboxGear::SignalNotAvaliable:
        case GearboxGear::Neutral:
        default:
            return false;
    }
}

const char* gear_to_text(GearboxGear g) {
    switch (g) {
        case GearboxGear::Reverse_First:
            return "R1";
        case GearboxGear::Reverse_Second:
            return "R2";
        case GearboxGear::First:
            return "D1";
        case GearboxGear::Second:
            return "D2";
        case GearboxGear::Third:
            return "D3";
        case GearboxGear::Fourth:
            return "D4";
        case GearboxGear::Fifth:
            return "D5";
        case GearboxGear::Park:
            return "P";
        case GearboxGear::SignalNotAvaliable:
            return "SNA";
        case GearboxGear::Neutral:
            return "N";
        default:
            return "";
    }
}

void Gearbox::inc_gear_request() {
    this->ask_upshift = true;
    this->ask_downshift = false;
}

void Gearbox::dec_gear_request() {
    this->ask_upshift = false;
    this->ask_downshift = true;
}

GearboxGear next_gear(GearboxGear g) {
    switch (g) {
        case GearboxGear::First:
            return GearboxGear::Second;
        case GearboxGear::Second:
            return GearboxGear::Third;
        case GearboxGear::Third:
            return GearboxGear::Fourth;
        case GearboxGear::Fourth:
            return GearboxGear::Fifth;
        case GearboxGear::Park:
        case GearboxGear::SignalNotAvaliable:
        case GearboxGear::Neutral:
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
        default:
            return g;
    }
}

GearboxGear prev_gear(GearboxGear g) {
    switch (g) {
        case GearboxGear::Second:
            return GearboxGear::First;
        case GearboxGear::Third:
            return GearboxGear::Second;
        case GearboxGear::Fourth:
            return GearboxGear::Third;
        case GearboxGear::Fifth:
            return GearboxGear::Fourth;
        case GearboxGear::First:
        case GearboxGear::Park:
        case GearboxGear::SignalNotAvaliable:
        case GearboxGear::Neutral:
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
        default:
            return g;
    }
}

#define SHIFT_TIMEOUT_MS 3000 // If a shift hasn't occurred after this much time, we assume shift has failed!

#ifdef BLUE_SOLENOIDS
    #define SHIFT_DELAY_MS 60 // 50% slower ramping for blue solenoids
#else
    #define SHIFT_DELAY_MS 40
#endif

/**
 * @brief Used to shift between forward gears
 * 
 * @param init_spc Initial SPC value
 * @param init_mpc Initial MPC value
 * @param shift_solenoid Shift solenoid to open up to do the shift
 * @param target_shift_duration_ms Target time in ms for the gear change to occur
 * @param targ_gear Target gear that we are shifting to
 * @return uint16_t - The actual time taken to shift gears. This is fed back into the adaptation network so it can better meet 'target_shift_duration_ms'
 */
ShiftResponse Gearbox::elapse_shift(ProfileGearChange req_lookup, AbstractProfile* profile, bool is_upshift) {
    SensorData pre_shift = this->sensor_data;
    ESP_LOGI("ELAPSE_SHIFT", "Shift started!");
    ShiftData sd = pressure_mgr->get_shift_data(&this->sensor_data, req_lookup, profile->get_shift_characteristics(req_lookup, &pre_shift));
    CLAMP(sd.shift_speed, 1, 10); // Ensure shift speed is a valid amount
    float curr_spc_pwm = sd.initial_spc_pwm;
    this->tcc->on_shift_start(sensor_data.current_timestamp_ms, !is_upshift, sd.shift_firmness, &pre_shift);
    sol_mpc->write_pwm_percent_with_voltage(sd.mpc_pwm, this->sensor_data.voltage);
    sd.shift_solenoid->write_pwm_percent_with_voltage(1000, this->sensor_data.voltage); // Start shifting
    sol_spc->write_pwm_percent_with_voltage(curr_spc_pwm, this->sensor_data.voltage); // Open SPC
    bool torque_reduction = false;
    bool torque_cut = false;
    if (profile == manual && this->sensor_data.pedal_pos > 170 && is_upshift) {
        egs_can_hal->set_torque_request(TorqueRequest::Minimum); // Test for manual upshift
        torque_reduction = true;
        torque_cut = true;
    }
    uint32_t elapsed = 0; // Counter for shift timing
    // Change gears begin  
    int max_d_rpm = 0;
    int min_d_rpm = 0;
    float avg_d_rpm = 0;
    bool first = false;
    int rpm_samples = 0;
    int start_rpm = this->sensor_data.input_rpm;  
    int last_rpm = this->sensor_data.input_rpm;
    bool shift_measure_complete = false;
    while(elapsed <= SHIFT_TIMEOUT_MS) {
        elapsed += SHIFT_DELAY_MS;
        vTaskDelay(SHIFT_DELAY_MS/portTICK_PERIOD_MS);
        // Measure RPM deltas
        // Measure D_RPM
        int new_rpm = this->sensor_data.input_rpm;
        int delta = new_rpm - last_rpm;
        avg_d_rpm += delta;
        if (first) {
            min_d_rpm = delta;
            max_d_rpm = delta;
            first = false;
        } else {
            if (delta < min_d_rpm) {
                min_d_rpm = delta;
            } else if (delta > max_d_rpm) {
                max_d_rpm = delta;
            }
        }
        rpm_samples += 1;
        last_rpm = new_rpm;


        if (this->sensor_data.input_rpm > start_rpm+70 && is_upshift) {
            this->flaring = true; // Upshift flare
        } else if (this->sensor_data.input_rpm < start_rpm-100 && !is_upshift) {
            this->flaring = true; // Downshift flare
        }
        sol_mpc->write_pwm_percent_with_voltage(sd.mpc_pwm, this->sensor_data.voltage);
        if (curr_spc_pwm > sd.shift_speed) {
            curr_spc_pwm -= sd.shift_speed;
        }
        sol_spc->write_pwm_percent_with_voltage(curr_spc_pwm, this->sensor_data.voltage); // Open SPC
        // Check using actual gear ratios (high speed moving, easiest way)
        if (this->est_gear_idx == 0 && torque_reduction && elapsed > 200) {
            torque_reduction = false;
            egs_can_hal->set_torque_request(TorqueRequest::None);
        }
        if (this->est_gear_idx == sd.targ_g) {
            shift_measure_complete = true;
            break;
        } else if (sensor_data.output_rpm < 100 && elapsed >= 1000) { // Fix for stationary shifts
            shift_measure_complete = false; // Cannot measure for adaptation (Standstill)
            break;
        }
    }
    if (avg_d_rpm != 0 && rpm_samples != 0) {
        avg_d_rpm /= (float)rpm_samples;
    }
    this->gear_disagree_count = 0;
    ESP_LOGI("ELAPSE_SHIFT", "SHIFT_END (Actual time %d ms - Target was %d d_rpm). DELTAS: (Max: %d, Min: %d, Avg: %.2f)", elapsed, sd.targ_d_rpm, max_d_rpm, min_d_rpm, avg_d_rpm);
    // Shift complete - Return the elapsed time for the shift to feedback into the adaptation system
    this->tcc->on_shift_complete(this->sensor_data.current_timestamp_ms);
    sol_spc->write_pwm_12_bit(0);
    vTaskDelay(100);
    sd.shift_solenoid->write_pwm_12_bit(0);
    sol_mpc->write_pwm_12_bit(0);
    this->shifting = false;
    this->flaring = false;
    bool shifted = true;
    if (sensor_data.output_rpm > 200 && this->est_gear_idx == sd.curr_g) { // WTF Didn't change gears!!!!
        shifted = false;
    }
    ShiftResponse response = {
        .shifted = shifted,
        .measure_ok = shift_measure_complete,
        .flared = flaring,
        .d_output_rpm = (int)this->sensor_data.output_rpm-(int)pre_shift.output_rpm,
        .torque_cut = torque_cut,
        .min_d_rpm = min_d_rpm,
        .max_d_rpm = max_d_rpm,
        .avg_d_rpm = (int)avg_d_rpm,
        .target_d_rpm = sd.targ_d_rpm,
    };
    if (this->pressure_mgr != nullptr) {
        this->pressure_mgr->perform_adaptation(&pre_shift, req_lookup, response, is_upshift);
    };
    return response;
}

void Gearbox::shift_thread() {
    this->shifting = true;
    GearboxGear curr_target = this->target_gear;
    GearboxGear curr_actual = this->actual_gear;
    uint16_t batt_voltage = 12000; // Assume 12V
    Sensors::read_vbatt(&batt_voltage);
    if (curr_actual == curr_target) {
        ESP_LOGW("SHIFTER", "Gears are the same????");
        goto cleanup;
    }
    if (!is_controllable_gear(curr_actual) && !is_controllable_gear(curr_target)) { // N->P or P->N
        sol_mpc->write_pwm_percent_with_voltage(333, sensor_data.voltage);
        sol_spc->write_pwm_percent_with_voltage(400, sensor_data.voltage); // 40%
        sol_y4->write_pwm_percent_with_voltage(200, sensor_data.voltage); // 3-4 is pulsed at 20%
        ESP_LOGI("SHIFTER", "No need to shift");
        this->actual_gear = curr_target; // Set on startup
        goto cleanup;
    } else if (is_controllable_gear(curr_actual) != is_controllable_gear(curr_target)) { // This would be a garage shift, either in or out
        ESP_LOGI("SHIFTER", "Garage shift");
        if (is_controllable_gear(curr_target)) {
            egs_can_hal->set_torque_request(TorqueRequest::Minimum); // Minimum torque during garage shifts
            // N/P -> R/D
            // Defaults (Start in 2nd)
            int spc_start = 400;
            int mpc_start = 800;
            int spc_ramp = 10;
            int mpc_ramp = 1;
            int y4_pwm_val = 800;

            if (!start_second) { // Starting in 1st
                mpc_start = 650;
                spc_start = 300;
                y4_pwm_val = 800;
                spc_ramp = 5;
            }
            sol_mpc->write_pwm_percent_with_voltage(mpc_start, sensor_data.voltage);
            sol_spc->write_pwm_percent_with_voltage(spc_start, sensor_data.voltage);
            sol_y4->write_pwm_percent_with_voltage(y4_pwm_val, sensor_data.voltage); // Full on
            while (spc_start > 50) {
                spc_start -= spc_ramp;
                mpc_start -= mpc_ramp;
                sol_spc->write_pwm_percent_with_voltage(spc_start, sensor_data.voltage);
                sol_mpc->write_pwm_percent_with_voltage(mpc_start, sensor_data.voltage);
                vTaskDelay(20/portTICK_PERIOD_MS);
            }
            vTaskDelay(200);
            sol_y4->write_pwm_percent(0);
            sol_spc->write_pwm_percent(0);
        } else {
            // Garage shifting to N or P, we can just set the pressure back to idle
            sol_spc->write_pwm_percent_with_voltage(400, sensor_data.voltage);
            sol_mpc->write_pwm_percent_with_voltage(330, sensor_data.voltage);
            sol_y4->write_pwm_percent_with_voltage(200, sensor_data.voltage); // Back to idle
        }
        if (is_fwd_gear(curr_target)) {
            // Last forward known gear
            // This way we better handle shifting from N->D at speed!
            this->actual_gear = this->last_fwd_gear;
        } else {
            this->actual_gear = curr_target; // R1/R2
        }
        goto cleanup;
    } else { // Both gears are controllable
        ESP_LOGI("SHIFTER", "Both gears are controllable");
        if (is_fwd_gear(curr_target) != is_fwd_gear(curr_actual)) {
            // In this case, we set the current gear to neutral, then thread will re-spawn
            ESP_LOGI("SHIFTER", "Shifter got stuck in R-D. Returning and trying again");
            this->actual_gear = GearboxGear::Neutral;
            goto cleanup;
        } else if (is_fwd_gear(curr_target) && is_fwd_gear(curr_actual)){
            // Forward shift logic
            if (curr_target > curr_actual) { // Upshifting
                ESP_LOGI("SHIFTER", "Upshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                //this->show_upshift = true;
                ProfileGearChange pgc;
                ShiftResponse response;
                if (curr_target == GearboxGear::Second) { // 1-2
                    pgc = ProfileGearChange::ONE_TWO;
                } else if (curr_target == GearboxGear::Third) { // 2-3
                    pgc = ProfileGearChange::TWO_THREE;
                } else if (curr_target == GearboxGear::Fourth) { // 3-4
                    pgc = ProfileGearChange::THREE_FOUR;
                } else if (curr_target == GearboxGear::Fifth) { // 4-5
                    pgc = ProfileGearChange::FOUR_FIVE;
                } else { // WTF
                    this->target_gear = this->actual_gear;
                    goto cleanup;
                }
                portENTER_CRITICAL(&this->profile_mutex);
                AbstractProfile* prof = this->current_profile;
                portEXIT_CRITICAL(&this->profile_mutex);
                response = elapse_shift(pgc, prof, true);
                this->actual_gear = curr_target;
                this->start_second = true;
                egs_can_hal->set_last_shift_time(abs(response.avg_d_rpm));
                goto cleanup;
            } else { // Downshifting
                ESP_LOGI("SHIFTER", "Downshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                //this->show_downshift = true;
                ProfileGearChange pgc;
                ShiftResponse resp;
                if (curr_target == GearboxGear::First) { // 2-1
                    pgc = ProfileGearChange::TWO_ONE;
                    this->start_second = false;
                } else if (curr_target == GearboxGear::Second) { // 3-2
                    pgc = ProfileGearChange::THREE_TWO;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Third) { // 4-3
                    pgc = ProfileGearChange::FOUR_THREE;
                } else if (curr_target == GearboxGear::Fourth) { // 5-4
                    pgc = ProfileGearChange::FIVE_FOUR;
                } else { // WTF
                    this->target_gear = this->actual_gear;
                    goto cleanup;
                }
                portENTER_CRITICAL(&this->profile_mutex);
                AbstractProfile* prof = this->current_profile;
                portEXIT_CRITICAL(&this->profile_mutex);
                resp = elapse_shift(pgc, prof, false);
                egs_can_hal->set_last_shift_time(abs(resp.avg_d_rpm));
                this->actual_gear = curr_target;
                goto cleanup;
            }
        } else {
            ESP_LOGI("SHIFTER", "Ignoring request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
        }
        goto cleanup;
    }
cleanup:
    ESP_LOGI("SHIFTER", "Shift complete");
    egs_can_hal->set_torque_request(TorqueRequest::None);
    egs_can_hal->set_requested_torque(0);
    this->shifting = false;
    vTaskDelete(nullptr);
}

void Gearbox::inc_subprofile() {
    portENTER_CRITICAL(&this->profile_mutex);
    if (this->current_profile != nullptr) {
        this->current_profile->increment_subprofile();
    }
    portEXIT_CRITICAL(&this->profile_mutex);
}

void Gearbox::controller_loop() {
    bool lock_state = false;
    ShifterPosition last_position = ShifterPosition::SignalNotAvaliable;
    ESP_LOGI("GEARBOX", "GEARBOX START!");
    uint64_t expire_check = esp_timer_get_time() + 100000; // 100ms
    bool lock = true; // P or N
    while (esp_timer_get_time() < expire_check) {
        this->shifter_pos = egs_can_hal->get_shifter_position_ewm(esp_timer_get_time()/1000, 250);
        last_position = this->shifter_pos;
        if (this->shifter_pos == ShifterPosition::P || this->shifter_pos == ShifterPosition::N) {
            break; // Default startup, OK
        } else if (this->shifter_pos == ShifterPosition::D) { // Car is in motion forwards!
            this->actual_gear = GearboxGear::Fifth;
            this->target_gear = GearboxGear::Fifth;
            this->gear_disagree_count = 20; // Set disagree counter to non 0. This way gearbox must calculate ratio
        } else if (this->shifter_pos == ShifterPosition::R) { // Car is in motion backwards!
            this->actual_gear = GearboxGear::Reverse_Second;
            this->target_gear = GearboxGear::Reverse_Second;
        }
        vTaskDelay(5);
    }

    uint64_t last_output_measure_time = esp_timer_get_time()/1000;
    int old_output_rpm = this->sensor_data.output_rpm;
    while(1) {
        uint64_t now = esp_timer_get_time()/1000;
        this->sensor_data.current_timestamp_ms = now;
        bool can_read = true;
        if (!this->calc_input_rpm(&sensor_data.input_rpm)) {
            can_read = false;
        }
        if(can_read && this->calc_output_rpm(&this->sensor_data.output_rpm, now)) {
            if (now - last_output_measure_time > 250) {
                this->sensor_data.d_output_rpm = this->sensor_data.output_rpm - old_output_rpm;
                old_output_rpm = this->sensor_data.output_rpm;
                last_output_measure_time = now;
            }
            if (!shifting && this->sensor_data.output_rpm > 300 && this->sensor_data.input_rpm > 300) {
                if (is_fwd_gear(this->actual_gear)) {
                    if (calcGearFromRatio(false) && this->est_gear_idx != 0) {
                        // Compare gears
                        GearboxGear estimate = gear_from_idx(this->est_gear_idx);
                        if (estimate != this->actual_gear) {
                            gear_disagree_count++;
                            if (gear_disagree_count >= 50) {
                                this->actual_gear = estimate; // DID NOT SHIFT!
                                this->target_gear = estimate;
                                this->last_fwd_gear = estimate;
                            }
                        } else {
                            gear_disagree_count = 0;
                        }
                    }
                } else {
                    gear_disagree_count = 0;
                }
            } else {
                gear_disagree_count = 0;
            }
        } else {
            can_read = false;
            gear_disagree_count = 0;
        }
        
        egs_can_hal->set_input_shaft_speed(this->sensor_data.input_rpm);
        if (can_read && this->sensor_data.output_rpm >= 100) {
            bool rev = !is_fwd_gear(this->target_gear);
            if (!this->calcGearFromRatio(rev)) {
                //ESP_LOGE("GEARBOX", "GEAR RATIO IMPLAUSIBLE");
            }
        }
        sensor_data.is_braking = egs_can_hal->get_is_brake_pressed(now, 250);
        this->sensor_data.engine_rpm = egs_can_hal->get_engine_rpm(now, 250);
        if (this->sensor_data.engine_rpm == UINT16_MAX) {
            this->sensor_data.engine_rpm = 0;
        }
        if (Sensors::parking_lock_engaged(&lock_state)) {
            egs_can_hal->set_safe_start(lock_state);
            this->shifter_pos = egs_can_hal->get_shifter_position_ewm(now, 250);
            if (
                this->shifter_pos == ShifterPosition::P ||
                this->shifter_pos == ShifterPosition::P_R ||
                this->shifter_pos == ShifterPosition::R ||
                this->shifter_pos == ShifterPosition::R_N ||
                this->shifter_pos == ShifterPosition::N ||
                this->shifter_pos == ShifterPosition::N_D ||
                this->shifter_pos == ShifterPosition::D
            ) {
                if (this->shifter_pos != last_position) {
                    if (lock_state) {
                        if (this->shifter_pos == ShifterPosition::P) {
                            this->target_gear = GearboxGear::Park;
                            last_position = this->shifter_pos;
                            if (this->tcc != nullptr) {
                                this->tcc->save_adaptation_data();
                            }
                            if (this->pressure_mgr != nullptr) {
                                this->pressure_mgr->save();
                            }
                        } else if (this->shifter_pos == ShifterPosition::N) {
                            this->target_gear = GearboxGear::Neutral;
                            last_position = this->shifter_pos;
                        }
                    } else {
                        // Drive or R!
                        if (this->shifter_pos == ShifterPosition::R || this->shifter_pos == ShifterPosition::R_N || this->shifter_pos == ShifterPosition::P_R) {
                            this->target_gear = this->start_second ? GearboxGear::Reverse_Second : GearboxGear::Reverse_First;
                            last_position = this->shifter_pos;
                        } else if (this->shifter_pos == ShifterPosition::D || this->shifter_pos == ShifterPosition::N_D) {
                            this->target_gear = this->start_second ? GearboxGear::Second : GearboxGear::First;
                            last_position = this->shifter_pos;
                        }
                    }
                }
            }
        }
        if (this->sensor_data.engine_rpm > 500) {
            if (!shifting && control_solenoids && is_controllable_gear(this->actual_gear)) {
                sol_mpc->write_pwm_12_bit(0);
                //sol_mpc->write_pwm_percent_with_voltage(pressure_mgr->get_mpc_active_duty_percent(), sensor_data.voltage);
            }
            if (is_fwd_gear(this->actual_gear)) {
                bool want_upshift = false;
                bool want_downshift = false;
                if (!shifting && this->actual_gear == this->target_gear && gear_disagree_count == 0) {
                    // Enter critical ISR section
                    portENTER_CRITICAL(&this->profile_mutex);
                    AbstractProfile* p = this->current_profile;
                    // Exit critical
                    portEXIT_CRITICAL(&this->profile_mutex);
                    // Check if profile is loaded
                    if (p != nullptr) {
                        // Ask the current drive profile if it thinks, given the current
                        // data, if the car should up/downshift
                        if(p->should_upshift(this->actual_gear, &this->sensor_data)) {
                            want_upshift = true;
                        }
                        if(p->should_downshift(this->actual_gear, &this->sensor_data)) {
                            want_downshift = true;
                        }
                    }
                    if (want_upshift && want_downshift) {
                        egs_can_hal->set_display_msg(GearboxMessage::ActuateParkingBreak); // WTF
                    } else if ((this->ask_upshift || want_upshift) && this->actual_gear < GearboxGear::Fifth) {
                        // Check RPMs
                        GearboxGear next = next_gear(this->actual_gear);
                        if (calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, next) > STALL_RPM+100) {
                            this->target_gear = next;
                            this->sensor_data.last_shift_time = now;
                        }
                    } else if ((this->ask_downshift || want_downshift) && this->actual_gear > GearboxGear::First) {
                        // Check RPMs
                        GearboxGear prev = prev_gear(this->actual_gear);
                        if (calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, prev) < REDLINE_RPM-400) {
                            this->target_gear = prev;
                            this->sensor_data.last_shift_time = now;
                        }
                    }
                    //this->ask_downshift = false;
                    //this->ask_upshift = false;
                }
                // Now, how do we prioritize up/downshift behaviour?
                // Upshifting when accelerating should take precidence.

                if (this->ask_upshift) {
                    this->ask_upshift = false;
                    if (this->actual_gear < GearboxGear::Fifth && this->target_gear == this->actual_gear && !shifting) {
                        this->target_gear = next_gear(this->actual_gear);
                    }
                } else if (this->ask_downshift) {
                    this->ask_downshift = false;
                    if (this->actual_gear > GearboxGear::First && this->target_gear == this->actual_gear && !shifting) {
                        this->target_gear = prev_gear(this->actual_gear);
                    }
                }
                if (can_read) {
                    uint8_t p_tmp = egs_can_hal->get_pedal_value(now, 100);
                    if (p_tmp != 0xFF) {
                        this->sensor_data.pedal_pos = p_tmp;
                    }
                    if (!Sensors::read_vbatt(&this->sensor_data.voltage)) {
                        this->sensor_data.voltage = 12000;
                    }

                    if (is_fwd_gear(this->actual_gear) && is_fwd_gear(this->target_gear)) {
                        this->sensor_data.tcc_slip_rpm = sensor_data.engine_rpm - sensor_data.input_rpm;
                        if (this->tcc != nullptr) {
                            this->tcc->update(this->target_gear, this->pressure_mgr, LockupType::Shut, &this->sensor_data, this->shifting);
                        }
                    } else {
                        this->tcc_percent = 0;
                        sol_tcc->write_pwm_12_bit(0);
                    }
                }
            }
            if (this->target_gear != this->actual_gear && this->shifting == false) {
                // Create shift task to change gears for us!
                xTaskCreatePinnedToCore(Gearbox::start_shift_thread, "Shift handler", 8192, this, 10, &this->shift_task, 1);
            }
        } else {
            if (control_solenoids) {
                sol_mpc->write_pwm_12_bit(0);
                sol_spc->write_pwm_12_bit(0);
                sol_tcc->write_pwm_12_bit(0);
                sol_y3->write_pwm_12_bit(0);
                sol_y4->write_pwm_12_bit(0);
                sol_y5->write_pwm_12_bit(0);
            } else {
                egs_can_hal->set_safe_start(false); // Inhibit engine starting when solenoids are not controlled!
            }
        }
        int tmp_atf = 0;
        if (!Sensors::read_atf_temp(&tmp_atf)) {
            // Default to engine coolant
            this->sensor_data.atf_temp = (egs_can_hal->get_engine_coolant_temp(now, 250));
        } else {
            this->sensor_data.atf_temp = tmp_atf;
        }
        egs_can_hal->set_gearbox_temperature(this->sensor_data.atf_temp);
        egs_can_hal->set_shifter_position(egs_can_hal->get_shifter_position_ewm(now, 250));

        egs_can_hal->set_target_gear(this->target_gear);
        egs_can_hal->set_actual_gear(this->actual_gear);
        egs_can_hal->set_solenoid_pwm(sol_y3->get_pwm() >> 4, SolenoidName::Y3);
        egs_can_hal->set_solenoid_pwm(sol_y4->get_pwm() >> 4, SolenoidName::Y4);
        egs_can_hal->set_solenoid_pwm(sol_y5->get_pwm() >> 4, SolenoidName::Y5);
        egs_can_hal->set_solenoid_pwm(sol_spc->get_pwm() >> 4, SolenoidName::SPC);
        egs_can_hal->set_solenoid_pwm(sol_mpc->get_pwm() >> 4, SolenoidName::MPC);
        egs_can_hal->set_solenoid_pwm(sol_tcc->get_pwm() >> 4, SolenoidName::TCC);

        int static_torque = egs_can_hal->get_static_engine_torque(now, 500);
        if (static_torque != INT_MAX) {
            this->sensor_data.static_torque = static_torque;
        }
        int max_torque = egs_can_hal->get_maximum_engine_torque(now, 500);
        if (max_torque != INT_MAX) {
            this->sensor_data.max_torque = max_torque;
        }
        int min_torque = egs_can_hal->get_minimum_engine_torque(now, 500);
        if (min_torque != INT_MAX) {
            this->sensor_data.min_torque = min_torque;
        }

        //ESP_LOGI("GEARBOX", "Torque: MIN: %3d, MAX: %3d, STAT: %3d", min_torque, max_torque, static_torque);
        // Show debug symbols on IC
        if (this->show_upshift && this->show_downshift) {
            egs_can_hal->set_display_msg(GearboxMessage::RequestGearAgain);
        } else if (this->show_upshift) {
            egs_can_hal->set_display_msg(GearboxMessage::Upshift); 
        } else if (this->show_downshift) {
            egs_can_hal->set_display_msg(GearboxMessage::Downshift);
        } else {
            egs_can_hal->set_display_msg(GearboxMessage::None);
        }
        if (is_fwd_gear(this->actual_gear)) {
            if (sensor_data.input_rpm > 1000) {
                float percent = (float)sensor_data.engine_rpm/(float)sensor_data.input_rpm;
                uint16_t innertia = TCC_INTERTIA_NM*percent;
                if (innertia > 0x3C) {
                    innertia = 0x3C;
                }
                egs_can_hal->set_turbine_torque_loss(innertia);
            } else {
                egs_can_hal->set_turbine_torque_loss(0);
            }
        } else {
            egs_can_hal->set_turbine_torque_loss(0);
        }

        // Lastly, set display gear
        portENTER_CRITICAL(&this->profile_mutex);
        if (this->current_profile != nullptr) {
            if (this->flaring) {
                // Takes president
                egs_can_hal->set_display_msg(GearboxMessage::None);
                egs_can_hal->set_display_gear(GearboxDisplayGear::Failure, false);
            } else {
                egs_can_hal->set_display_gear(this->current_profile->get_display_gear(this->target_gear, this->actual_gear), this->current_profile == manual);
            }
        }
        portEXIT_CRITICAL(&this->profile_mutex);
        vTaskDelay(20/portTICK_PERIOD_MS); // 50 updates/sec!
    }
}

bool Gearbox::calc_input_rpm(int* dest) {
    RpmReading rpm{};
    bool ok = false;
    bool conduct_sanity_check = gear_disagree_count == 0 && (
        (this->actual_gear == GearboxGear::Second && this->target_gear == GearboxGear::Second) ||
        (this->actual_gear == GearboxGear::Third && this->target_gear == GearboxGear::Third) ||
        (this->actual_gear == GearboxGear::Fourth && this->target_gear == GearboxGear::Fourth)
    );
    if (Sensors::read_input_rpm(&rpm, conduct_sanity_check)) {
        ok = true;
        this->sensor_data.input_rpm = rpm.calc_rpm;
    }
    return ok;
}

bool Gearbox::calc_output_rpm(int* dest, uint64_t now) {
    WheelData left = egs_can_hal->get_rear_left_wheel(now, 250);
    WheelData right = egs_can_hal->get_rear_right_wheel(now, 250);
    //ESP_LOGI("WRPM","R:(%d %d) L:(%d %d)", (int)right.current_dir, right.double_rpm, (int)left.current_dir, left.double_rpm);
    float rpm = 0;
    if (left.current_dir == WheelDirection::SignalNotAvaliable && right.current_dir == WheelDirection::SignalNotAvaliable) {
        //ESP_LOGE("CALC_OUTPUT_RPM", "Could not obtain right and left wheel RPM!");
        return false;
    } else if (left.current_dir == WheelDirection::SignalNotAvaliable) { // Right OK
        ESP_LOGW("CALC_OUTPUT_RPM", "Could not obtain left wheel RPM, trusting the right one!");
        rpm = right.double_rpm;
    } else if (right.current_dir == WheelDirection::SignalNotAvaliable) { // Left OK
        ESP_LOGW("CALC_OUTPUT_RPM", "Could not obtain right wheel RPM, trusting the left one!");
        rpm = left.double_rpm;
    } else { // Both sensors OK!
        rpm = abs(left.double_rpm) > abs(right.double_rpm) ? left.double_rpm : right.double_rpm;
    }
    rpm *= diff_ratio_f;
    rpm /= 2;
    *dest = rpm;
    return true;
}

bool Gearbox::calcGearFromRatio(bool is_reverse) {
    float ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
    if (is_reverse) {
        ratio *=-1;
        for(uint8_t i = 0; i < 2; i++) { // Scan the 2 reverse gears
            GearRatioLimit limits = GEAR_RATIO_LIMITS[i+5];
            if (ratio >= limits.min && ratio <= limits.max) {
                this->est_gear_idx = i+1;
                return true;
            }
        }
    } else {
        for(uint8_t i = 0; i < 5; i++) { // Scan the 5 forwards gears
            GearRatioLimit limits = GEAR_RATIO_LIMITS[i];
            if (ratio > limits.min && ratio < limits.max) {
                this->est_gear_idx = i+1;
                return true;
            }
        }
    }
    this->est_gear_idx = 0;
    return false;
}