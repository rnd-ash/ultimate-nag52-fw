#include "gearbox.h"
#include "scn.h"


const float diff_ratio_f = (float)DIFF_RATIO / 1000.0;

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

void Gearbox::shift_thread() {
    this->shifting = true;
    GearboxGear curr_target = this->target_gear;
    GearboxGear curr_actual = this->actual_gear;
    if (curr_actual == curr_target) {
        goto cleanup;  
    }
    ESP_LOGI("SHIFTER", "CURR: %s, TARG: %s", gear_to_text(curr_actual), gear_to_text(curr_target));
    if (!is_controllable_gear(curr_actual) && !is_controllable_gear(curr_target)) {
        sol_mpc->write_pwm_percent(333); // 33%
        sol_spc->write_pwm_percent(400); // 40%
        sol_y4->write_pwm_percent(500); // 3-4 is pulsed at 50%
        ESP_LOGI("SHIFTER", "No need to shift");
        this->actual_gear = curr_target; // Set on startup
        goto cleanup;
    } else if (is_controllable_gear(curr_actual) == !is_controllable_gear(curr_target)) { // This would be a garage shift, either in or out
        ESP_LOGI("SHIFTER", "Garage shift");
        sol_spc->write_pwm_percent(400);
        sol_y4->write_pwm_percent(500);
        vTaskDelay(700);
        if (is_controllable_gear(curr_target)) {
            sol_spc->write_pwm_percent(0);
            sol_mpc->write_pwm_percent(0);
            sol_y4->write_pwm_percent(0);
        } else {
            sol_spc->write_pwm_percent(400);
            sol_mpc->write_pwm_percent(333);
            sol_y4->write_pwm_percent(500); // Back to idle
        }
        this->actual_gear = curr_target; // Set on startup
        goto cleanup;
    }
cleanup:
    ESP_LOGI("SHIFTER", "Shift complete");
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
    // Before we enter, we have to check what gear we are in as the 'actual gear'
    ESP_LOGI("GEARBOX", "GEARBOX START!");
    while(1) {
        this->calc_input_rpm(&rpm);
        if (Sensors::parking_lock_engaged(&lock_state)) {
            egs_can_hal->set_safe_start(lock_state);
            ShifterPosition pos = egs_can_hal->get_shifter_position_ewm();
            if (lock_state) {
                if (pos == ShifterPosition::P) {
                    this->target_gear = GearboxGear::Park;
                } else if (pos == ShifterPosition::N) {
                    this->target_gear = GearboxGear::Neutral;
                }
            } else {
                // Drive or R!
                if (pos == ShifterPosition::R) {
                    this->target_gear = GearboxGear::Reverse_Second;
                } else if (pos == ShifterPosition::D) {
                    this->target_gear = GearboxGear::Second;
                }
            }
        }
        if (rpm > 500) {
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
        if (!Sensors::read_atf_temp(&atf_temp)) {
            // Default to engine coolant
            atf_temp = (egs_can_hal->get_engine_coolant_temp())*10;
        }
        egs_can_hal->set_gearbox_temperature(atf_temp/10);
        egs_can_hal->set_shifter_position(egs_can_hal->get_shifter_position_ewm());

        egs_can_hal->set_target_gear(this->target_gear);
        egs_can_hal->set_actual_gear(this->actual_gear);
        // Lastly, set display gear
        portENTER_CRITICAL(&this->profile_mutex);
        if (this->current_profile != nullptr) {
            egs_can_hal->set_display_gear(this->current_profile->get_display_gear(this->target_gear, this->actual_gear));
        }
        portEXIT_CRITICAL(&this->profile_mutex);

        vTaskDelay(10); // 100 updates/sec!
    }
}

bool Gearbox::calc_input_rpm(uint32_t* dest) {
    // Compare N2 and N3 sensors based on our TARGET gear
    uint32_t tmp = 0;
    switch (this->target_gear) {
        case GearboxGear::First:
        case GearboxGear::Fifth:
            tmp = Sensors::read_n2_rpm();
            if (tmp > OVERSPEED_RPM) {
                ESP_LOGE("CALC_INPUT_RPM", "N2 overspeed detected!");
                return false;
            } else {
                *dest = tmp;
                return true;
            }
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            tmp = Sensors::read_n3_rpm();
            if (tmp > OVERSPEED_RPM) {
                ESP_LOGE("CALC_INPUT_RPM", "N3 overspeed detected!");
                return false;
            } else {
                *dest = tmp;
                return true;
            }
        default:
            // Compare both!
            uint32_t n2 = Sensors::read_n2_rpm();
            uint32_t n3 = Sensors::read_n3_rpm();
            if (n2 > OVERSPEED_RPM || n3 > OVERSPEED_RPM) {
                ESP_LOGE("CALC_INPUT_RPM", "N2 or N3 overspeed detected!");
                return false;
            }
            // Rational check
            if (this->target_gear == this->actual_gear) { // Only perform rational check if we are defiantly in the right gear!
                if (n3 > n2) {
                    if (n3-n2 > 250) { // Rational check
                        ESP_LOGE("CALC_INPUT_RPM", "Rational check failed! N3-N2 delta is %d", n3-n2);
                        return false;
                    }
                } else if (n2 > n3) {
                    if (n2-n3 > 250) { // Rational check
                        ESP_LOGE("CALC_INPUT_RPM", "Rational check failed! N2-N3 delta is %d", n2-n3);
                        return false;
                    }
                }
            }
            *dest = (n2+n3)/2;
            return true;
    }
}

bool Gearbox::calc_output_rpm(int* dest) {
    WheelData left = egs_can_hal->get_rear_left_wheel();
    WheelData right = egs_can_hal->get_rear_right_wheel();
    //ESP_LOGI("WRPM","R:(%d %d) L:(%d %d)", (int)right.current_dir, right.double_rpm, (int)left.current_dir, left.double_rpm);
    int rpm = 0;
    if (left.current_dir == WheelDirection::SignalNotAvaliable && right.current_dir == WheelDirection::SignalNotAvaliable) {
        ESP_LOGE("CALC_OUTPUT_RPM", "Could not obtain right and left wheel RPM!");
        return false;
    } else if (left.current_dir == WheelDirection::SignalNotAvaliable) { // Right OK
        ESP_LOGW("CALC_OUTPUT_RPM", "Could not obtain left wheel RPM, trusting the right one!");
        rpm = right.double_rpm;
        if (right.current_dir == WheelDirection::Stationary) {
            rpm = 0;
        } else if (right.current_dir == WheelDirection::Reverse) {
            rpm *= -1;
        }
    } else if (right.current_dir == WheelDirection::SignalNotAvaliable) { // Left OK
        ESP_LOGW("CALC_OUTPUT_RPM", "Could not obtain right wheel RPM, trusting the left one!");
        rpm = left.double_rpm;
        if (left.current_dir == WheelDirection::Stationary) {
            rpm = 0;
        } else if (left.current_dir == WheelDirection::Reverse) {
            rpm *= -1;
        }
    } else { // Both sensors OK!
        // Return max value
        if (left.current_dir == WheelDirection::Stationary) {
            left.double_rpm = 0;
        } else if (left.current_dir == WheelDirection::Reverse) {
            left.double_rpm *= -1;
        }
        if (right.current_dir == WheelDirection::Stationary) {
            right.double_rpm = 0;
        } else if (right.current_dir == WheelDirection::Reverse) {
            right.double_rpm *= -1;
        }
        rpm = abs(left.double_rpm) > abs(right.double_rpm) ? left.double_rpm : right.double_rpm;
    }
    //rpm /= diff_ratio_f;
    *dest = rpm;
    return true;
}

