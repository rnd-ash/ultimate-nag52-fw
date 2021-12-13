#include "gearbox.h"
#include "scn.h"
#include "nvs/eeprom_config.h"

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

void place_in_map(int input_rpm, int pedal, int atf_temp, uint16_t shift_time_ms, SHIFT_LAYER* dest) {
    // Dest -> [ATF_TEMP_STAGES][MAX_RPM-MIN_RPM][PEDAL_STEPS]
    int temp_idx = (atf_temp/10)-1;
    CLAMP(temp_idx, 0, ATF_TEMP_STAGES);

    int rpm_idx = ((input_rpm) / RPM_STEP) - MIN_RPM - 1;
    CLAMP(rpm_idx, 0, MAX_RPM);

    int ped_idx = (pedal/PEDAL_STEPS) - 1;
    CLAMP(ped_idx, 0, PEDAL_STEPS);
    ESP_LOGI("PLACE_IN_MAP", "Setting value %d at [%d][%d][%d]", shift_time_ms, temp_idx, rpm_idx, ped_idx);
    *dest[temp_idx][rpm_idx][ped_idx] = shift_time_ms;
}

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
    };
}

void Gearbox::set_profile(AbstractProfile* prof) {
    if (prof != nullptr) { // Only change if not nullptr!
        portENTER_CRITICAL(&this->profile_mutex);
        this->current_profile = prof;
        // Set CAN display
        egs_can_hal->set_drive_profile(this->current_profile->get_profile());
        this->min_fwd_gear = prof->get_start_gear();
        portEXIT_CRITICAL(&this->profile_mutex);
    }
}

bool Gearbox::start_controller() {
    xTaskCreatePinnedToCore(Gearbox::start_controller_internal, "GEARBOX", 32768, (void*)this, 10, nullptr, 1);
    return true;
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
        case GearboxGear::Sixth:
        case GearboxGear::Seventh:
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
        case GearboxGear::Sixth:
        case GearboxGear::Seventh:
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
        case GearboxGear::Sixth:
            return "D6";
        case GearboxGear::Seventh:
            return "D7";
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
        case GearboxGear::Fifth:
            return GearboxGear::Sixth;
        case GearboxGear::Sixth:
            return GearboxGear::Seventh;
        case GearboxGear::Seventh:
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
        case GearboxGear::Sixth:
            return GearboxGear::Fifth;
        case GearboxGear::Seventh:
            return GearboxGear::Sixth;
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

#define POST_SHIFT_FADE_TIME_MS 1000 // On-off solenoid time to prevent water hammer effect
#define SHIFT_TIMEOUT_MS 3000 // If a shift hasn't occured after this much time, we assume shift has failed!
#define SHIFT_DELAY_MS 10
#define SHIFT_CONFIRM_COUNT 200/SHIFT_DELAY_MS // 200ms in gear for confirm

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
uint16_t Gearbox::elapse_shift(uint16_t init_spc, uint16_t init_mpc, Solenoid* shift_solenoid, uint16_t target_shift_duration_ms, uint8_t curr_gear, uint8_t targ_gear) {
    ESP_LOGI("SHIFTER", "SHIFT START %d -> %d, SPC: %d, MPC: %d", curr_gear, targ_gear, init_spc, init_mpc);
    uint8_t confirm_count = 0; // Conformations we are in the target gear
    uint16_t spc = init_spc;
    uint16_t mpc = init_mpc;
    sol_mpc->write_pwm_percent(spc); // Set initial SPC/MPC values
    sol_spc->write_pwm_percent(mpc);
    vTaskDelay(100);
    shift_solenoid->write_pwm_percent_with_voltage(1000, this->sensor_data.voltage); // Don't burn out the solenoid!
    if (targ_gear < curr_gear && this->tcc_percent > 100) {
        this->tcc_percent = 90;
        sol_tcc->write_pwm_percent_with_voltage(this->tcc_percent, this->sensor_data.voltage); // Smooth downshifts
    }
    uint32_t elapsed = 0; // Counter for shift timing
    // Change gears begin
    bool confirm_shift = false;
    int start_rpm = this->sensor_data.engine_rpm;
    while(elapsed <= SHIFT_TIMEOUT_MS) {
        if (this->sensor_data.engine_rpm > start_rpm+100 && targ_gear > curr_gear) {
            // FLARING!
            this->flaring = true;
        }

        // Check using actual gear ratios (high speed moving, easiest way)
        if (this->est_gear_idx == targ_gear) {
            confirm_count++;
        }
        if (confirm_count >= SHIFT_CONFIRM_COUNT) { // Yay - gear shift confirmed!
            confirm_shift = true;
            break;
        }
        elapsed += SHIFT_DELAY_MS;
        vTaskDelay(SHIFT_DELAY_MS);
    }
    ESP_LOGI("ELAPSE_SHIFT", "SHIFT_END (TO %d) (Actual time %d ms - Target was %d ms)", targ_gear, elapsed, target_shift_duration_ms);
    // Shift complete - Return the elapsed time for the shift to feedback into the adaptation system
    shift_solenoid->write_pwm(0);
    sol_spc->write_pwm(0);
    while(mpc > 10) {
        if (mpc >= 10) {
            mpc -= 5;
        }
        sol_mpc->write_pwm_percent(mpc);
        vTaskDelay(10);
    }
    sol_mpc->write_pwm(0);
    this->shifting = false;
    this->flaring = false;
    return elapsed;
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
        sol_mpc->write_pwm_percent(333); // 33%
        sol_spc->write_pwm_percent(400); // 40%
        sol_y4->write_pwm_percent(400); // 3-4 is pulsed at 20%
        ESP_LOGI("SHIFTER", "No need to shift");
        this->actual_gear = curr_target; // Set on startup
        goto cleanup;
    } else if (is_controllable_gear(curr_actual) != is_controllable_gear(curr_target)) { // This would be a garage shift, either in or out
        ESP_LOGI("SHIFTER", "Garage shift");
    #define SPC_START_PERC_2 40
    #define SPC_START_PERC_1 40
        if (is_controllable_gear(curr_target)) {
            if (this->start_second) { // Second gear garage shift
                sol_spc->write_pwm_percent(SPC_START_PERC_2*10); // Decrease SPC
                sol_mpc->write_pwm_percent(100); // Increase MPC pressure to keep B2 clutch in suspension
                sol_y4->write_pwm_percent_with_voltage(1000, sensor_data.voltage); // Full on
                vTaskDelay(300);
                // Slowly ramp up SPC pressure again
                for (int i = SPC_START_PERC_2; i > 0; i--) {
                    sol_spc->write_pwm_percent(10*i);
                    vTaskDelay(20);
                }
            } else { // First gear garage shift
                sol_spc->write_pwm_percent(SPC_START_PERC_1*10); // Decrease SPC
                sol_mpc->write_pwm_percent(50); // Increase MPC pressure to keep B2 clutch in suspension
                sol_y4->write_pwm_percent_with_voltage(1000, sensor_data.voltage); // Full on
                vTaskDelay(300);
                // Slowly ramp up SPC pressure again
                for (int i = SPC_START_PERC_1; i > 0; i--) {
                    sol_spc->write_pwm_percent(10*i);
                    vTaskDelay(20);
                }
            }
            sol_y4->write_pwm_percent(0);
            sol_spc->write_pwm_percent(0);
            sol_mpc->write_pwm_percent(0);
        } else {
            // Garage shifting to N or P, we can just set the pressure back to idle
            sol_spc->write_pwm_percent(400);
            sol_mpc->write_pwm_percent(333);
            sol_y4->write_pwm_percent(400); // Back to idle
        }
        this->actual_gear = curr_target; // and we are in gear!
        goto cleanup;
    } else { // Both gears are controllable
        ESP_LOGI("SHIFTER", "Both gears are controllable");
        if (is_fwd_gear(curr_target) != is_fwd_gear(curr_actual)) {
            // In this case, we set the current gear to neutral, then thread will re-spawn
            ESP_LOGI("SHIFTER", "Shifter got stuck in R-D. Returning and trying again");
            this->actual_gear = GearboxGear::Neutral;
            goto cleanup;
        } else if (is_fwd_gear(curr_target) && is_fwd_gear(curr_actual)){
            int t1 = this->sensor_data.input_rpm;
            int t2 = this->sensor_data.pedal_pos;
            int t3 = this->sensor_data.atf_temp;
            // Forward shift logic
            if (curr_target > curr_actual) { // Upshifting
                ESP_LOGI("SHIFTER", "Upshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                this->map_changes_pending = true;
                this->show_upshift = true;
                ProfileGearChange pgc;
                uint16_t time;
                ShiftData sd;
                Solenoid* sol;
                uint8_t cur_g, tar_g;
                if (curr_target == GearboxGear::Second) { // 1-2
                    pgc = ProfileGearChange::ONE_TWO;
                    cur_g = 1; tar_g = 2;
                    sol = sol_y3;
                } else if (curr_target == GearboxGear::Third) { // 2-3
                    pgc = ProfileGearChange::TWO_THREE;
                    cur_g = 2; tar_g = 3;
                    sol = sol_y5;
                } else if (curr_target == GearboxGear::Fourth) { // 3-4
                    pgc = ProfileGearChange::THREE_FOUR;
                    cur_g = 3; tar_g = 4;
                    sol = sol_y4;
                } else if (curr_target == GearboxGear::Fifth) { // 4-5
                    pgc = ProfileGearChange::FOUR_FIVE;
                    cur_g = 4; tar_g = 5;
                    sol = sol_y3;
                } else { // WTF
                    this->target_gear = this->actual_gear;
                    goto cleanup;
                }
                portENTER_CRITICAL(&this->profile_mutex);
                if (this->current_profile != nullptr) {
                    sd = this->current_profile->get_shift_data(pgc, &this->sensor_data);
                } else {
                    sd = ShiftData { .spc_perc = 200, .mpc_perc = 200, .targ_ms = 500 };
                }
                portEXIT_CRITICAL(&this->profile_mutex);
                time = elapse_shift(sd.spc_perc, sd.mpc_perc, sol, sd.targ_ms, cur_g, tar_g);
                egs_can_hal->set_last_shift_time(time);
                this->actual_gear = curr_target;
                this->start_second = true;
                goto cleanup;
            } else { // Downshifting
                ESP_LOGI("SHIFTER", "Downshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                this->map_changes_pending = true;
                this->show_downshift = true;
                if (curr_target == GearboxGear::First) { // Test 2->1
                    uint16_t time = elapse_shift(250, 250,sol_y3, 1000, 2, 1);
                    egs_can_hal->set_last_shift_time(time);
                    place_in_map(t1,t2,t3, time, two_downto_one);
                    this->actual_gear = curr_target;
                    this->start_second = false;
                } else if (curr_target == GearboxGear::Second) { // Test 3->2
                    uint16_t time = elapse_shift(240, 230, sol_y5, 1000, 3, 2);
                    egs_can_hal->set_last_shift_time(time);
                    place_in_map(t1,t2,t3, time, three_downto_two);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Third) { // Test 4->3
                    uint16_t time = elapse_shift(340, 320, sol_y4, 1000, 4, 3);
                    egs_can_hal->set_last_shift_time(time);
                    place_in_map(t1,t2,t3, time, four_downto_three);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Fourth) { // Test 5->4
                    uint16_t time = elapse_shift(300, 300, sol_y3, 1000, 5, 4);
                    egs_can_hal->set_last_shift_time(time);
                    place_in_map(t1,t2,t3, time, five_downto_four);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else {
                    this->target_gear = this->actual_gear; // IGNORE
                }
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
    this->show_downshift = false;
    this->show_upshift = false;
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
    // Before we enter, we have to check what gear we are in as the 'actual gear'
    ESP_LOGI("GEARBOX", "GEARBOX START!");
    while(1) {
        uint64_t now = esp_timer_get_time();
        bool can_read = this->calc_input_rpm(&this->sensor_data.input_rpm) && this->calc_output_rpm(&this->sensor_data.output_rpm, now);
        egs_can_hal->set_input_shaft_speed(this->sensor_data.input_rpm);
        if (can_read && this->sensor_data.output_rpm >= 100) {
            bool rev = !is_fwd_gear(this->target_gear);
            if (!this->calcGearFromRatio(rev)) {
                //ESP_LOGE("GEARBOX", "GEAR RATIO IMPLAUSIBLE");
            }
        }
        bool brake = egs_can_hal->get_is_brake_pressed(now, 250);
        this->sensor_data.engine_rpm = egs_can_hal->get_engine_rpm(now, 250);
        if (this->sensor_data.engine_rpm == UINT16_MAX) {
            this->sensor_data.engine_rpm = 0;
        }
        if (Sensors::parking_lock_engaged(&lock_state)) {
            egs_can_hal->set_safe_start(lock_state);
            ShifterPosition pos = egs_can_hal->get_shifter_position_ewm(now, 250);
            if (
                pos == ShifterPosition::P || // Only obide by definitive positions for now, no intermittent once
                pos == ShifterPosition::R ||
                pos == ShifterPosition::N ||
                pos == ShifterPosition::D
            ) {
                if (pos != last_position) {
                    if (lock_state) {
                        if (pos == ShifterPosition::P) {
                            this->target_gear = GearboxGear::Park;
                            last_position = pos;
                            // Save state
                            if (this->map_changes_pending) {
                                EEPROM::save_map_data();
                                this->map_changes_pending = false;
                            }
                        } else if (pos == ShifterPosition::N) {
                            this->target_gear = GearboxGear::Neutral;
                            last_position = pos;
                        }
                    } else {
                        // Drive or R!
                        if (pos == ShifterPosition::R) {
                            this->target_gear = this->start_second ? GearboxGear::Reverse_Second : GearboxGear::Reverse_First;
                            last_position = pos;
                        } else if (pos == ShifterPosition::D) {
                            this->target_gear = this->start_second ? GearboxGear::Second : GearboxGear::First;
                            last_position = pos;
                        }
                    }
                }
            }
        }
        if (this->sensor_data.engine_rpm > 500) {
            if (is_fwd_gear(this->actual_gear)) {
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
                    if (
                        this->sensor_data.output_rpm < 5 && brake && sensor_data.pedal_pos > 0 && this->actual_gear == GearboxGear::First && sensor_data.engine_rpm > 1500) {
                        egs_can_hal->set_race_start(true);
                    } else {
                        egs_can_hal->set_race_start(false);
                    }
                    uint8_t p_tmp = egs_can_hal->get_pedal_value(now, 100);
                    if (p_tmp != 0xFF) {
                        this->sensor_data.pedal_pos = p_tmp;
                    }
                    if (!Sensors::read_vbatt(&this->sensor_data.voltage)) {
                        this->sensor_data.voltage = 12000;
                    }

                    if (is_fwd_gear(this->actual_gear) && this->sensor_data.input_rpm >= 1000) {
                        if (this->tcc_percent < 90) {
                            this->tcc_percent = 90;
                        } else {
                            float inc = 0;
                            int slip = abs(sensor_data.engine_rpm - sensor_data.input_rpm);
                            if (this->tcc_percent < 150) {
                                if (slip > 500) {
                                    inc = 0.25;
                                    if (sensor_data.pedal_pos/100.0 > inc) {
                                        inc = sensor_data.pedal_pos/100.0;
                                    }
                                } else if (slip > 200) {
                                    inc = 0.1;
                                    if (sensor_data.pedal_pos/250.0 > inc) {
                                        inc = sensor_data.pedal_pos/250.0;
                                    }
                                }
                            }
                            if (this->tcc_percent > 90 && slip < 50 && slip > 0 && sensor_data.pedal_pos > 100 && sensor_data.engine_rpm < 2000) {
                                inc = -0.1;
                            }
                            this->tcc_percent += inc;
                            sol_tcc->write_pwm_percent_with_voltage(this->tcc_percent, sensor_data.voltage);
                        }
                    } else {
                        this->tcc_percent = 0;
                        sol_tcc->write_pwm(0);
                    }
                }
            }
            if (this->target_gear != this->actual_gear && this->shifting == false) {
                // Create shift task to change gears for us!
                xTaskCreatePinnedToCore(Gearbox::start_shift_thread, "Shift handler", 8192, this, 10, &this->shift_task, 1);
            }
        } else {
            sol_mpc->write_pwm(0);
            sol_spc->write_pwm(0);
            sol_tcc->write_pwm(0);
            sol_y3->write_pwm(0);
            sol_y4->write_pwm(0);
            sol_y5->write_pwm(0);
        }
        if (!Sensors::read_atf_temp(&this->sensor_data.atf_temp)) {
            // Default to engine coolant
            this->sensor_data.atf_temp = (egs_can_hal->get_engine_coolant_temp(now, 250));
        }
        egs_can_hal->set_gearbox_temperature(this->sensor_data.atf_temp);
        egs_can_hal->set_shifter_position(egs_can_hal->get_shifter_position_ewm(now, 250));

        egs_can_hal->set_target_gear(this->target_gear);
        egs_can_hal->set_actual_gear(this->actual_gear);
        /*
        if (is_fwd_gear(this->actual_gear)) {
            GearboxGear n = next_gear(this->actual_gear);
            GearboxGear p = prev_gear(this->actual_gear);
            ESP_LOGI(
                "FUTURE CALC", "Curr gear %s, (Prev %s: %d IN RPM), (Next %s: %d)", 
                gear_to_text(this->actual_gear),
                gear_to_text(p),
                calc_input_rpm_from_req_gear(sensor_data.output_rpm, p),
                gear_to_text(n),
                calc_input_rpm_from_req_gear(sensor_data.output_rpm, n)
            );
        }
        */
        egs_can_hal->set_solenoid_pwm(sol_y3->get_pwm(), SolenoidName::Y3);
        egs_can_hal->set_solenoid_pwm(sol_y4->get_pwm(), SolenoidName::Y4);
        egs_can_hal->set_solenoid_pwm(sol_y5->get_pwm(), SolenoidName::Y5);
        egs_can_hal->set_solenoid_pwm(sol_spc->get_pwm(), SolenoidName::SPC);
        egs_can_hal->set_solenoid_pwm(sol_mpc->get_pwm(), SolenoidName::MPC);
        egs_can_hal->set_solenoid_pwm(sol_tcc->get_pwm(), SolenoidName::TCC);

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
        if (this->show_upshift) {
            egs_can_hal->set_display_msg(GearboxMessage::Upshift); 
        } else if (this->show_downshift) {
            egs_can_hal->set_display_msg(GearboxMessage::Downshift);
        } else {
            egs_can_hal->set_display_msg(GearboxMessage::None);
        }

        // Lastly, set display gear
        portENTER_CRITICAL(&this->profile_mutex);
        if (this->current_profile != nullptr) {
            if (this->flaring) {
                // Takes president
                egs_can_hal->set_display_msg(GearboxMessage::None);
                egs_can_hal->set_display_gear('F');
            } else {
                egs_can_hal->set_display_gear(this->current_profile->get_display_gear(this->target_gear, this->actual_gear));
            }
        }
        portEXIT_CRITICAL(&this->profile_mutex);
        vTaskDelay(50); // 20 updates/sec!
    }
}

bool Gearbox::calc_input_rpm(int* dest) {
    uint32_t n2 = Sensors::read_n2_rpm();
    if (n2 < 50) { // Skip erroneous pulses
        n2 = 0;
    }
    uint32_t n3 = Sensors::read_n3_rpm();
    if (n3 < 50) { // Skip erroneous pulses
        n3 = 0;
    }
    // Compare N2 and N3 sensors based on our TARGET gear
    if (this->actual_gear == GearboxGear::Neutral || this->actual_gear == GearboxGear::Park) { 
        if (n3 < 100 && n2 != 0) {
            *dest = n2 * 1.64;
            return true;
        } else {
            *dest = (n2+n3)/2;
            return true;
        }
    }
    switch (this->target_gear) {
        case GearboxGear::First:
        case GearboxGear::Fifth:
            n2 *= 1.64;
            if (n2 > OVERSPEED_RPM) {
                //ESP_LOGE("CALC_INPUT_RPM", "N2 overspeed detected!");
                return false;
            } else {
                *dest = n2;
                return true;
            }
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            if (n3 > OVERSPEED_RPM) {
                //ESP_LOGE("CALC_INPUT_RPM", "N3 overspeed detected!");
                return false;
            } else {
                *dest = n3;
                return true;
            }
        default:
            // Compare both!
            if (n2 > OVERSPEED_RPM || n3 > OVERSPEED_RPM) {
                //ESP_LOGE("CALC_INPUT_RPM", "N2 or N3 overspeed detected!");
                return false;
            }
            // Rational check
            if (this->target_gear == this->actual_gear) { // Only perform rational check if we are defiantly in the right gear!
                if (n3 > n2) {
                    if (n3-n2 > 250) { // Rational check
                        //ESP_LOGE("CALC_INPUT_RPM", "Rational check failed! N3-N2 delta is %d", n3-n2);
                        return false;
                    }
                } else if (n2 > n3) {
                    if (n2-n3 > 250) { // Rational check
                        //ESP_LOGE("CALC_INPUT_RPM", "Rational check failed! N2-N3 delta is %d", n2-n3);
                        return false;
                    }
                }
            }
            *dest = (n2+n3)/2;
            return true;
    }
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