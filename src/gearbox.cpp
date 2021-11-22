#include "gearbox.h"
#include "scn.h"
#include "nvs/eeprom_config.h"

const float diff_ratio_f = (float)DIFF_RATIO / 1000.0;


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

Gearbox::Gearbox() {
    this->current_profile = nullptr;
    egs_can_hal->set_drive_profile(GearboxProfile::Underscore); // Uninitialized
    this->profile_mutex = portMUX_INITIALIZER_UNLOCKED;
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
    uint16_t v = 12000;
    float spc = init_spc;
    float mpc = init_mpc;
    float d_spc = spc / ((float)POST_SHIFT_FADE_TIME_MS/SHIFT_DELAY_MS);
    float d_mpc = mpc / ((float)POST_SHIFT_FADE_TIME_MS/SHIFT_DELAY_MS);
    if (!Sensors::read_vbatt(&v)) {
        v = 12000; // Could not read voltage, assume 12.0V
    }
    uint8_t confirm_count = 0; // Conformations we are in the target gear
    sol_mpc->write_pwm_percent(init_spc); // Set initial SPC/MPC values
    sol_spc->write_pwm_percent(init_mpc);
    shift_solenoid->write_pwm_percent_with_voltage(1000, v); // Don't burn out the solenoid!
    sol_tcc->write_pwm(0); // Unlock TCC here (Reduces flaring to do this later than setting SPC and MPC values)
    uint32_t elapsed = 0; // Counter for shift timing
    // Change gears begin
    bool confirm_shift = false;
    while(elapsed <= SHIFT_TIMEOUT_MS) {
        // Check using actual gear ratios (high speed moving, easiest way)
        if (this->est_gear_idx == targ_gear) {
            confirm_count++;
        } else { // Maybe car is stationary / low speed? Then we can't use gear ratios
            // We can instead check using the following ways (Only if car is moving slowly - Not stationary!)
            //    1. When going from 1->2, N3 RPM will go from 0 RPM to same RPM as N2 (Which itself will speed up by .3x)! 
            //    2. When going from 2->1, N3 RPM will stop reporting, and N2 will report 0.64x real speed
            // 
            // If the car is stationary, what do we do???? - For now use hardcoded shift times and hope it actually changed gears!
            /*
            uint32_t n2 = Sensors::read_n2_rpm();
            uint32_t n3 = Sensors::read_n3_rpm();
            if (curr_gear == 2 && targ_gear == 1) {
                // Down into 1 (N3 should stop, N2 should report)
                if (n2 != 0 && n3 == 0) { confirm_count++; }
            } else if (curr_gear == 1 && targ_gear == 2) {
                // Up into 2 (Should read the same)
                if (n2 != 0 && n3 != 0 && abs((int)n2-(int)n3) < 100) { confirm_count++; }
            }
            */

        }
        if (confirm_count >= 3) { // Yay - gear shift confirmed!
            confirm_shift = true;
            break;
        }
        elapsed += SHIFT_DELAY_MS;
        vTaskDelay(SHIFT_DELAY_MS);
    }
    // Fade out SPC and MPC (EGS52 does this in 1 second)
    //while(spc > 1 && mpc > 1) {
    //    spc -= d_spc;
    //    mpc -= d_mpc;
    //    sol_mpc->write_pwm_percent(mpc);
    //    sol_spc->write_pwm_percent(spc);
    //    vTaskDelay(SHIFT_DELAY_MS);
    //}
    ESP_LOGI("ELAPSE_SHIFT", "SHIFT_END (TO %d) (Actual time %d ms - Target was %d ms)", targ_gear, elapsed, target_shift_duration_ms);
    // Shift complete - Return the elapsed time for the shift to feedback into the adaptation system
    shift_solenoid->write_pwm(0);
    sol_mpc->write_pwm(0);
    sol_spc->write_pwm(0);
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
        sol_y4->write_pwm_percent(300); // 3-4 is pulsed at 20%
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
                sol_y4->write_pwm_percent(1000); // Full on
                vTaskDelay(500);
                // Slowly ramp up SPC pressure again
                for (int i = SPC_START_PERC_2; i > 0; i--) {
                    sol_spc->write_pwm_percent(10*i);
                    vTaskDelay(30);
                }
            } else { // First gear garage shift
                sol_spc->write_pwm_percent(SPC_START_PERC_1*10); // Decrease SPC
                sol_mpc->write_pwm_percent(50); // Increase MPC pressure to keep B2 clutch in suspension
                sol_y4->write_pwm_percent(1000); // Full on
                vTaskDelay(500);
                // Slowly ramp up SPC pressure again
                for (int i = SPC_START_PERC_1; i > 0; i--) {
                    sol_spc->write_pwm_percent(10*i);
                    vTaskDelay(30);
                }
            }
            sol_spc->write_pwm_percent(0);
            sol_mpc->write_pwm_percent(0);
            sol_y4->write_pwm_percent(0);
        } else {
            // Garage shifting to N or P, we can just set the pressure back to idle
            sol_spc->write_pwm_percent(400);
            sol_mpc->write_pwm_percent(333);
            sol_y4->write_pwm_percent(300); // Back to idle
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
            // Forward shift logic
            if (curr_target > curr_actual) { // Upshifting
                ESP_LOGI("SHIFTER", "Upshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                this->map_changes_pending = true;
                if (curr_target == GearboxGear::Second) { // Test 1->2
                    uint16_t time = elapse_shift(250, 250, sol_y3, 1000, 1, 2);
                    place_in_map(this->input_rpm, this->pedal_pos, this->atf_temp, time, one_to_two);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Third) { // Test 2->3
                    uint16_t time = elapse_shift(350, 320, sol_y5, 1000, 2, 3);
                    place_in_map(this->input_rpm, this->pedal_pos, this->atf_temp, time, two_to_three);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Fourth) { // Test 3->4
                    uint16_t time = elapse_shift(300, 250, sol_y4, 1000, 3, 4);
                    place_in_map(this->input_rpm, this->pedal_pos, this->atf_temp, time, three_to_four);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Fifth) { // Test 4->5
                    uint16_t time = elapse_shift(300, 300, sol_y3, 1000, 4, 5);
                    place_in_map(this->input_rpm, this->pedal_pos, this->atf_temp, time, four_to_five);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else {
                    this->target_gear = this->actual_gear; // IGNORE
                }
                goto cleanup;
            } else { // Downshifting
                ESP_LOGI("SHIFTER", "Downshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                this->map_changes_pending = true;
                if (curr_target == GearboxGear::First) { // Test 2->1
                    uint16_t time = elapse_shift(250, 250,sol_y3, 1000, 2, 1);
                    place_in_map(this->input_rpm, this->pedal_pos, this->atf_temp, time, two_downto_one);
                    this->actual_gear = curr_target;
                    this->start_second = false;
                } else if (curr_target == GearboxGear::Second) { // Test 3->2
                    elapse_shift(150, 150, sol_y5, 1000, 3, 2);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Third) { // Test 4->3
                    elapse_shift(300, 300, sol_y4, 1000, 4, 3);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Fourth) { // Test 5->4
                    elapse_shift(400, 400, sol_y3, 1000, 5, 4);
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
    int output_rpm = 0;
    int engine_rpm = 0;
    uint16_t voltage;
    ShifterPosition last_position = ShifterPosition::SignalNotAvaliable;
    // Before we enter, we have to check what gear we are in as the 'actual gear'
    ESP_LOGI("GEARBOX", "GEARBOX START!");
    while(1) {
        uint64_t now = esp_timer_get_time();
        bool can_read = this->calc_input_rpm(&this->input_rpm) && this->calc_output_rpm(&output_rpm, now);
        egs_can_hal->set_input_shaft_speed(this->input_rpm);
        if (can_read && output_rpm >= 100) {
            bool rev = !is_fwd_gear(this->target_gear);
            if (!this->calcGearFromRatio(this->input_rpm, output_rpm, rev)) {
                //ESP_LOGE("GEARBOX", "GEAR RATIO IMPLAUSIBLE");
            }
        }
        engine_rpm = egs_can_hal->get_engine_rpm(now, 250);
        if (engine_rpm == UINT16_MAX) {
            engine_rpm = 0;
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
        if (engine_rpm > 500) {
            if (is_fwd_gear(this->actual_gear)) {
                if (this->ask_upshift) {
                    ESP_LOGI("SHIFTER", "UP");
                    this->ask_upshift = false;
                    if (this->actual_gear < GearboxGear::Fifth && this->target_gear == this->actual_gear && !shifting) {
                        this->target_gear = next_gear(this->actual_gear);
                    }
                } else if (this->ask_downshift) {
                    ESP_LOGI("SHIFTER", "DN");
                    this->ask_downshift = false;
                    if (this->actual_gear > GearboxGear::First && this->target_gear == this->actual_gear && !shifting) {
                        this->target_gear = prev_gear(this->actual_gear);
                    }
                }
                if (can_read) {
                    uint8_t p_tmp = egs_can_hal->get_pedal_value(now, 100);
                    if (p_tmp != 0xFF) {
                        this->pedal_pos = p_tmp;
                    }
                    if (!Sensors::read_vbatt(&voltage)) {
                        voltage = 12000;
                    }

                    if (is_fwd_gear(this->actual_gear) && this->input_rpm >= 1200) {
                        sol_tcc->write_pwm_percent(100); // 15% ON
                    } else {
                        sol_tcc->write_pwm(0);
                    }
                        /*
                        //ESP_LOGI("TCC", "SLIP %d RPM. TCC %u mA (PWM %u/255 @ %d mV). Pedal %u. Gear %u", engine_rpm - input_rpm, sol_tcc->get_current_estimate(), sol_tcc->get_pwm(), voltage ,pedal, this->est_gear_idx);
                        if (engine_rpm <= 1000) { // Min working speed
                            this->tcc_percent = 0;
                            sol_tcc->write_pwm_percent(0);
                        } else if (this->actual_gear != GearboxGear::First) {
                            int rpm_diff = abs(engine_rpm - input_rpm);
                            // Do magical calculation to work out correct lockup        
                            if (pedal > 160 && this->tcc_percent > 2 && rpm_diff < 100) { // High load - unlock
                                this->tcc_percent-=2;
                            } else {
                                if (engine_rpm > input_rpm) {
                                    if (rpm_diff > 200 && this->tcc_percent < 333) { // Slip zone
                                        this->tcc_percent++;
                                    } else if (rpm_diff > 25 && this->tcc_percent < 500) { // Lock zone
                                        this->tcc_percent++;
                                    }
                                }
                            }
                            //sol_tcc->write_pwm_percent_with_voltage(this->tcc_percent, voltage);
                        }
                        */
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
        if (!Sensors::read_atf_temp(&this->atf_temp)) {
            // Default to engine coolant
            this->atf_temp = (egs_can_hal->get_engine_coolant_temp(now, 250))*10;
        }
        egs_can_hal->set_gearbox_temperature(atf_temp/10);
        egs_can_hal->set_shifter_position(egs_can_hal->get_shifter_position_ewm(now, 250));

        egs_can_hal->set_target_gear(this->target_gear);
        egs_can_hal->set_actual_gear(this->actual_gear);
        // Lastly, set display gear
        portENTER_CRITICAL(&this->profile_mutex);
        if (this->current_profile != nullptr) {
            egs_can_hal->set_display_gear(this->current_profile->get_display_gear(this->target_gear, this->actual_gear));
        }
        portEXIT_CRITICAL(&this->profile_mutex);
        vTaskDelay(25); // 40 updates/sec!
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

bool Gearbox::calcGearFromRatio(uint32_t input_rpm, uint32_t output_rpm, bool is_reverse) {
    float ratio = (float)input_rpm / (float)output_rpm;
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