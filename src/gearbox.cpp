#include "gearbox.h"
#include "scn.h"


const float diff_ratio_f = (float)DIFF_RATIO / 1000.0;

#define X_SIZE 12
#define Y_SIZE 14

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

typedef struct {
    uint8_t mpc;
    uint8_t spc;
} PressurePercentages;

PressurePercentages find_pressure_settings(const PressureMap spc_map, const PressureMap mpc_map, uint8_t pedal_position, int atf_temp) {
    if (atf_temp < -20) {
        atf_temp = -20;
    } else if (atf_temp > 100) {
        atf_temp = 100;
    } else {
        atf_temp = abs((atf_temp % 10) - atf_temp);
    }
    if (pedal_position > 100) {
        pedal_position = 100;
    } else {
        pedal_position = abs((pedal_position % 10) - pedal_position);
    }

    int y_coord = (atf_temp/10) + 2;
    int x_coord = (pedal_position/10)+1;
    if (y_coord < 0) {
        y_coord = 0;
    } else if (y_coord >= 14) {
        y_coord = 13;
    }

    if (x_coord < 0) {
        x_coord = 0;
    } else if (x_coord >= 12) {
        x_coord = 11;
    }

    uint8_t spc = spc_map[y_coord][x_coord];
    uint8_t mpc = mpc_map[y_coord][x_coord];


    ESP_LOGI("FPS", "ATF: %d C, PEDAL: %d %% = [%u,%u]", atf_temp, pedal_position, x_coord, y_coord);
    ESP_LOGI("FPS", "SPC: %u %%, MPC: %u %%", 100-spc, 100-mpc);
    return PressurePercentages {
        .mpc = mpc,
        .spc = spc,
    };

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
                if (curr_target == GearboxGear::Second) { // Test 1->2
                    //egs_can_hal->set_torque_request(TorqueRequest::Maximum);
                    //egs_can_hal->set_requested_torque(0);
                    sol_y3->write_pwm_percent(1000);
                    sol_spc->write_pwm_percent(400);
                    sol_mpc->write_pwm_percent(400);
                    for(int i = 40; i > 0; i--) {
                        sol_spc->write_pwm_percent(10*i);
                        sol_mpc->write_pwm_percent(10*i);
                        vTaskDelay(50);
                    }
                    sol_y3->write_pwm_percent(0);
                    sol_spc->write_pwm_percent(0);
                    sol_mpc->write_pwm_percent(0);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Third) { // Test 2->3
                    sol_y5->write_pwm_percent(1000);
                    sol_spc->write_pwm_percent(400);
                    sol_mpc->write_pwm_percent(400);
                    for(int i = 40; i > 0; i--) {
                        sol_spc->write_pwm_percent(10*i);
                        sol_mpc->write_pwm_percent(10*i);
                        vTaskDelay(50);
                    }
                    sol_y5->write_pwm_percent(0);
                    sol_spc->write_pwm_percent(0);
                    sol_mpc->write_pwm_percent(0);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Fourth) { // Test 3->4
                    sol_y4->write_pwm_percent(1000);
                    sol_spc->write_pwm_percent(400);
                    sol_mpc->write_pwm_percent(400);
                    for(int i = 40; i > 0; i--) {
                        sol_spc->write_pwm_percent(10*i);
                        sol_mpc->write_pwm_percent(10*i);
                        vTaskDelay(50);
                    }
                    sol_y4 ->write_pwm_percent(0);
                    sol_spc->write_pwm_percent(0);
                    sol_mpc->write_pwm_percent(0);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Fifth) { // Test 4->5
                    sol_y3->write_pwm_percent(1000);
                    sol_spc->write_pwm_percent(400);
                    sol_mpc->write_pwm_percent(400);
                    for(int i = 40; i > 0; i--) {
                        sol_spc->write_pwm_percent(10*i);
                        sol_mpc->write_pwm_percent(10*i);
                        vTaskDelay(50);
                    }
                    sol_y3->write_pwm_percent(0);
                    sol_spc->write_pwm_percent(0);
                    sol_mpc->write_pwm_percent(0);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else {
                    this->target_gear = this->actual_gear; // IGNORE
                }
                goto cleanup;
            } else { // Downshifting
                ESP_LOGI("SHIFTER", "Downshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                if (curr_target == GearboxGear::First) { // Test 2->1
                    sol_y3->write_pwm_percent(1000);
                    sol_spc->write_pwm_percent(400);
                    sol_mpc->write_pwm_percent(400);
                    for(int i = 40; i > 0; i--) {
                        sol_spc->write_pwm_percent(10*i);
                        sol_mpc->write_pwm_percent(10*i);
                        vTaskDelay(50);
                    }
                    sol_y3->write_pwm_percent(0);
                    sol_spc->write_pwm_percent(0);
                    sol_mpc->write_pwm_percent(0);
                    this->actual_gear = curr_target;
                    this->start_second = false;
                } else if (curr_target == GearboxGear::Second) { // Test 3->2
                    sol_y5->write_pwm_percent(1000);
                    sol_spc->write_pwm_percent(800); // Beefy
                    sol_mpc->write_pwm_percent(800); // Beefy
                    for(int i = 80; i > 0; i--) {
                        sol_spc->write_pwm_percent(10*i);
                        sol_mpc->write_pwm_percent(10*i);
                        vTaskDelay(25);
                    }
                    sol_y5->write_pwm_percent(0);
                    sol_spc->write_pwm_percent(0);
                    sol_mpc->write_pwm_percent(0);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Third) { // Test 4->3
                    sol_y4->write_pwm_percent(1000);
                    sol_spc->write_pwm_percent(400);
                    sol_mpc->write_pwm_percent(400);
                    for(int i = 40; i > 0; i--) {
                        sol_spc->write_pwm_percent(10*i);
                        sol_mpc->write_pwm_percent(10*i);
                        vTaskDelay(50);
                    }
                    sol_y4->write_pwm_percent(0);
                    sol_spc->write_pwm_percent(0);
                    sol_mpc->write_pwm_percent(0);
                    this->actual_gear = curr_target;
                    this->start_second = true;
                } else if (curr_target == GearboxGear::Fourth) { // Test 5->4
                    sol_y3->write_pwm_percent(1000);
                    sol_spc->write_pwm_percent(400);
                    sol_mpc->write_pwm_percent(400);
                    for(int i = 40; i > 0; i--) {
                        sol_spc->write_pwm_percent(10*i);
                        sol_mpc->write_pwm_percent(10*i);
                        vTaskDelay(50);
                    }
                    sol_y3->write_pwm_percent(0);
                    sol_spc->write_pwm_percent(0);
                    sol_mpc->write_pwm_percent(0);
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
    vTaskDelay(500); // Prevent over shifting!
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
    int atf_temp = 0;
    uint32_t rpm = 0;
    uint32_t output_rpm = 0;
    uint16_t eng_rpm = 0;
    uint8_t pedal = 0;
    uint16_t voltage;
    ShifterPosition last_position = ShifterPosition::SignalNotAvaliable;
    // Before we enter, we have to check what gear we are in as the 'actual gear'
    ESP_LOGI("GEARBOX", "GEARBOX START!");
    while(1) {
        uint64_t now = esp_timer_get_time();
        bool can_read = this->calc_input_rpm(&rpm) && this->calc_output_rpm(&output_rpm, now);
        egs_can_hal->set_input_shaft_speed(rpm);
        if (can_read && output_rpm >= 100) {
            bool rev = !is_fwd_gear(this->target_gear);
            if (!this->calcGearFromRatio(rpm, output_rpm, rev)) {
                //ESP_LOGE("GEARBOX", "GEAR RATIO IMPLAUSIBLE");
            }
        }
        eng_rpm = egs_can_hal->get_engine_rpm(now, 250);
        if (eng_rpm == UINT16_MAX) {
            eng_rpm = 0;
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
        if (eng_rpm > 500) {
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
                uint16_t stat = egs_can_hal->get_static_engine_torque(now, 100);
                uint16_t max = egs_can_hal->get_maximum_engine_torque(now, 100);
                uint16_t min = egs_can_hal->get_minimum_engine_torque(now, 100);
                //ESP_LOGI("ENG_PERF", "Pedal %u/250 @ %u RPM - STATIC: %u Nm, MIN: %u Nm, MAX: %u Nm", pedal, eng_rpm, stat, min, max);
                if (can_read) {
                    uint8_t p_tmp = egs_can_hal->get_pedal_value(now, 100);
                    if (p_tmp != 0xFF) {
                        pedal = p_tmp;
                    }
                    if (!Sensors::read_vbatt(&voltage)) {
                        voltage = 12000;
                    }
                    
                    // Clutch actuation logic :p
                    // Need engine and input RPM to be > 1000, not shifting and ensure current gear is fwd!

                    if (rpm > 1100 && eng_rpm > 1100 && !shifting && (this->actual_gear == GearboxGear::First || this->actual_gear == GearboxGear::Second)) {
                        if (pedal == 0) {
                            sol_tcc->write_pwm_percent_with_voltage(400, voltage);  // No load lock only has to be 40%
                        } else if (pedal < 100) { // 35%
                            if (abs((int)eng_rpm-(int)rpm) < 200) {
                                sol_tcc->write_pwm_percent_with_voltage(666, voltage);  // 66% Locked torque converter (0-100 RPM slip)
                            } else {
                                sol_tcc->write_pwm_percent(0);
                            }
                        } else {
                            sol_tcc->write_pwm_percent(0); // Unlock!
                        }
                        // TODO - Does doing 100% affect the solenoid??
                    } else {
                        sol_tcc->write_pwm_percent(0); // Unlock
                    }

                    //ESP_LOGI("TCC", "SLIP %d RPM. TCC %u mA (PWM %u/255 @ %d mV). Pedal %u. Gear %u", eng_rpm - rpm, sol_tcc->get_current_estimate(), sol_tcc->get_pwm(), voltage ,pedal, this->est_gear_idx);
                }
                else if (rpm < STALL_RPM && this->actual_gear > this->min_fwd_gear) { // Downshift
                    this->target_gear = prev_gear(this->actual_gear);
                }
            }
            if (this->target_gear != this->actual_gear && this->shifting == false) {
                // Create shift task to change gears for us!
                //sol_tcc->write_pwm_percent(0);
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
        if (!Sensors::read_atf_temp(&atf_temp)) {
            // Default to engine coolant
            atf_temp = (egs_can_hal->get_engine_coolant_temp(now, 250))*10;
        }
        this->temp_raw = atf_temp;
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
        vTaskDelay(20); // 50 updates/sec!
    }
}

bool Gearbox::calc_input_rpm(uint32_t* dest) {
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

bool Gearbox::calc_output_rpm(uint32_t* dest, uint64_t now) {
    WheelData left = egs_can_hal->get_rear_left_wheel(now, 250);
    WheelData right = egs_can_hal->get_rear_right_wheel(now, 250);
    //ESP_LOGI("WRPM","R:(%d %d) L:(%d %d)", (int)right.current_dir, right.double_rpm, (int)left.current_dir, left.double_rpm);
    float rpm = 0;
    if (left.current_dir == WheelDirection::SignalNotAvaliable && right.current_dir == WheelDirection::SignalNotAvaliable) {
        ESP_LOGE("CALC_OUTPUT_RPM", "Could not obtain right and left wheel RPM!");
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
                ESP_LOGI("CGFR","RATIO SAYS GEAR R%d (%f:1)", i+1, ratio);
                return true;
            }
        }
    } else {
        for(uint8_t i = 0; i < 5; i++) { // Scan the 2 reverse gears
            GearRatioLimit limits = GEAR_RATIO_LIMITS[i];
            if (ratio >= limits.min && ratio <= limits.max) {
                this->est_gear_idx = i+1;
                ESP_LOGI("CGFR","RATIO SAYS GEAR %d (%f:1)", i+1, ratio);
                return true;
            }
        }
    }
    this->est_gear_idx = 0;
    return false;
}