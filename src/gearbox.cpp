#include "gearbox.h"
#include "scn.h"
#include "nvs/eeprom_config.h"
#include "adv_opts.h"
#include <tcm_maths.h>
#include "solenoids/constant_current.h"
#include "speaker.h"

uint16_t Gearbox::redline_rpm = 4000;

#define max(a,b) \
  ({ \
    typeof (a) _a = (a); \
    typeof (b) _b = (b); \
    _a > _b ? _a : _b; \
  })

// ONLY FOR FORWARD GEARS!
int calc_input_rpm_from_req_gear(int output_rpm, GearboxGear req_gear, FwdRatios ratios) {
    switch (req_gear) {
        case GearboxGear::First:
            return output_rpm*ratios[0];
        case GearboxGear::Second:
            return output_rpm*ratios[1];
        case GearboxGear::Third:
            return output_rpm*ratios[2];
        case GearboxGear::Fourth:
            return output_rpm*ratios[3];
        case GearboxGear::Fifth:
            return output_rpm*ratios[4];
        default:
            return output_rpm;
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
    this->shift_reporter = new ShiftReporter();

    if(VEHICLE_CONFIG.is_large_nag) {
        this->gearboxConfig = GearboxConfiguration {
            .max_torque = MAX_TORQUE_RATING_NM_LARGE,
            .bounds = GEAR_RATIO_LIMITS_LARGE,
            .ratios = RATIOS_LARGE,
        };
    } else {
        this->gearboxConfig = GearboxConfiguration {
            .max_torque = MAX_TORQUE_RATING_NM_SMALL,
            .bounds = GEAR_RATIO_LIMITS_SMALL,
            .ratios = RATIOS_SMALL,
        };
    }
    this->pressure_mgr = new PressureManager(&this->sensor_data, this->gearboxConfig.max_torque);
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "---GEARBOX INFO---");
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Max torque: %d Nm", this->gearboxConfig.max_torque);
    for (int i = 0; i < 7; i++) {
        char c = i < 5 ? 'D' : 'R';
        int g = i < 5 ? i+1 : i-4;
        ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Gear ratio %c%d %.2f. Bounds: (%.2f - %.2f)", c, g, gearboxConfig.ratios[i], gearboxConfig.bounds[i].max, gearboxConfig.bounds[i].min);
    }
    if (VEHICLE_CONFIG.engine_type == 1) {
        this->redline_rpm = VEHICLE_CONFIG.red_line_rpm_petrol;
    } else {
        this->redline_rpm = VEHICLE_CONFIG.red_line_rpm_diesel;
    }
    if (this->redline_rpm < 4000) {
        this->redline_rpm = 4000; // just in case
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEAROX", "---OTHER CONFIG---");
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Redline RPM: %d", this->redline_rpm);
    this->diff_ratio_f = (float)VEHICLE_CONFIG.diff_ratio/1000.0;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Diff ratio: %.2f", this->diff_ratio_f);
}

void Gearbox::set_profile(AbstractProfile* prof) {
    if (prof != nullptr) { // Only change if not nullptr!
        portENTER_CRITICAL(&this->profile_mutex);
        this->current_profile = prof;
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

int find_target_ratio(int targ_gear, FwdRatios ratios) {
    if (targ_gear >= 1 && targ_gear <= 5) {
        return ratios[targ_gear-1]*100;
    } else {
        return 100; // Invalid
    }
}

#define SHIFT_TIMEOUT_MS 3000 // If a shift hasn't occurred after this much time, we assume shift has failed!
#define SHIFT_DELAY_MS 10 // 10ms steps


/**
 * @brief Used to shift between forward gears
 *
 * @return uint16_t - The actual time taken to shift gears. This is fed back into the adaptation network so it can better meet 'target_shift_duration_ms'
 */
void Gearbox::elapse_shift(ProfileGearChange req_lookup, AbstractProfile* profile, bool is_upshift) {
    SensorData pre_shift = this->sensor_data;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ELAPSE_SHIFT", "Shift started!");
    ShiftCharacteristics chars = profile->get_shift_characteristics(req_lookup, &this->sensor_data);
    ShiftData sd = pressure_mgr->get_shift_data(&this->sensor_data, req_lookup, chars, gearboxConfig.max_torque, this->mpc_working);
    bool gen_report = sensor_data.output_rpm > 100;
    ShiftReport dest_report;
    memset(&dest_report, 0x00, sizeof(ShiftReport));
    
    if (gen_report) {
        // Copy data
        dest_report.hold1_data = sd.hold1_data;
        dest_report.hold2_data = sd.hold2_data;
        dest_report.hold3_data = sd.hold3_data;
        dest_report.torque_data = sd.torque_data;
        dest_report.overlap_data = sd.overlap_data;
        dest_report.max_pressure_data = sd.max_pressure_data;
        dest_report.atf_temp_c = sensor_data.atf_temp;
        dest_report.initial_mpc_pressure = this->mpc_working;
        dest_report.flare_timestamp = 0;
        dest_report.requested_torque = UINT16_MAX; // Max if no request
        if (profile == nullptr) {
            dest_report.profile = 0xFF;
        } else {
            dest_report.profile = profile->get_profile_id();
        }
    }
    
    
    uint16_t index = 0;
    bool add_report = true;
    uint16_t stage_elapsed = 0;
    uint16_t total_elapsed = 0;
    uint16_t ss_open = 0;
    uint8_t shift_stage = SHIFT_PHASE_HOLD_1;
    // Open SPC for bleeding (Start of phase 1)
    float curr_spc_pressure = 0;
    uint16_t curr_sd_pwm = 0;
    pressure_mgr->set_target_spc_pressure(curr_spc_pressure);
    ShiftPhase* curr_phase = &sd.hold1_data;
    float ramp = ((float)(curr_phase->spc_pressure)-curr_spc_pressure) / ((float)MAX(curr_phase->ramp_time,SHIFT_DELAY_MS)/(float)SHIFT_DELAY_MS);
    float ramp_mpc = ((float)(curr_phase->mpc_pressure)-this->mpc_working) / ((float)MAX(curr_phase->ramp_time,SHIFT_DELAY_MS)/(float)SHIFT_DELAY_MS);
    
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT", "Shift start phase hold 1");
    uint16_t start_ratio = sensor_data.gear_ratio*100;
    bool shift_in_progress = false;
    while(true) {
        this->shift_stage = shift_stage;
        // Execute the current phase
        if (curr_phase != nullptr) {
            if (stage_elapsed < curr_phase->ramp_time && curr_spc_pressure < curr_phase->spc_pressure) {
                curr_spc_pressure += ramp;
                this->mpc_working += ramp_mpc;
                this->is_ramp = true;
            } else {
                this->mpc_working = curr_phase->mpc_pressure;
                this->is_ramp = false;
                // Holding
                curr_spc_pressure = curr_phase->spc_pressure;
                if (stage_elapsed >= curr_phase->ramp_time+curr_phase->hold_time) {
                    // Fire next phase!
                    stage_elapsed = 0;
                    // Current phase, so what do we do before firing the next phase?
                    if (shift_stage == SHIFT_PHASE_HOLD_1) {
                        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT", "Shift start phase hold 2");
                        // Going to hold 2
                        // open partial the shift solenoid
                        shift_stage = SHIFT_PHASE_HOLD_2;
                        curr_phase = &sd.hold2_data;
                        curr_sd_pwm = 4096;
                        start_ratio = sensor_data.gear_ratio*100; // Shift valve opens here so now take note of start ratio
                    } else if (shift_stage == SHIFT_PHASE_HOLD_2) {
                        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT", "Shift start phase hold 3");
                        shift_stage = SHIFT_PHASE_HOLD_3;
                        curr_phase = &sd.hold3_data;
                    } else if (shift_stage == SHIFT_PHASE_HOLD_3) {
                        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT", "Shift start phase torque");
                        shift_stage = SHIFT_PHASE_TORQUE;
                        curr_phase = &sd.torque_data;
                    } else if (shift_stage == SHIFT_PHASE_TORQUE) {
                        if (sensor_data.static_torque > 0 && this->est_gear_idx == sd.curr_g) {
                            float torque_amount = sd.torque_down_amount * (float)sensor_data.static_torque;
                            dest_report.requested_torque = torque_amount;
                            egs_can_hal->set_torque_request(TorqueRequest::Exact);
                            egs_can_hal->set_requested_torque(torque_amount);
                        }
                        if(this->est_gear_idx == sd.curr_g) {
                            this->tcc->on_shift_start(sensor_data.current_timestamp_ms, !is_upshift, &sensor_data, chars.shift_speed);
                        }
                        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT", "Shift start phase overlap");
                        shift_stage = SHIFT_PHASE_OVERLAP;
                        curr_phase = &sd.overlap_data;
                    } else if (shift_stage == SHIFT_PHASE_OVERLAP) {
                        egs_can_hal->set_torque_request(TorqueRequest::None);
                        egs_can_hal->set_requested_torque(0);
                        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT", "Shift start phase max pressure");
                        sd.max_pressure_data.mpc_pressure = pressure_mgr->find_working_mpc_pressure(this->actual_gear, &sensor_data, gearboxConfig.max_torque);
                        // No solenoids to touch, just inc counter
                        shift_stage = SHIFT_PHASE_MAX_P;
                        curr_phase = &sd.max_pressure_data;
                    } else if (shift_stage == SHIFT_PHASE_MAX_P) {
                        shift_stage++;
                        // We are done! return
                        curr_phase = nullptr;
                    }
                    if (curr_phase != nullptr) {
                        ramp = ((float)(curr_phase->spc_pressure)-curr_spc_pressure) / ((float)MAX(curr_phase->ramp_time,SHIFT_DELAY_MS)/(float)SHIFT_DELAY_MS);
                        ramp_mpc = ((float)(curr_phase->mpc_pressure)-this->mpc_working) / ((float)MAX(curr_phase->ramp_time,SHIFT_DELAY_MS)/(float)SHIFT_DELAY_MS);
                    }
                }
            }
        } else {
            // We are waiting for gearbox to complete
            if (this->est_gear_idx == sd.targ_g) {
                break; // Done!
            } else if (sensor_data.output_rpm < 100 && stage_elapsed >= 1000) {
                break;
            }
            if (stage_elapsed > SHIFT_TIMEOUT_MS) { // Too long at max p and car didn't shift!?
                dest_report.shift_timeout = 1;
                break;
            }
        }
        uint16_t ratio_now = sensor_data.gear_ratio*100;
        if (gen_report) {
            // Shift monitoring
            if (is_upshift) {
                if (ratio_now > start_ratio+10) { // Upshift - Ratio should get smaller so inverse means flaring)
                    if (!this->flaring) {
                        dest_report.flare_timestamp = total_elapsed; // Mark report where flare occurred
                    }
                    this->flaring = true;
                } else if (ratio_now < start_ratio-10) {
                    if (!shift_in_progress) {
                        if (gen_report) {
                            dest_report.transition_start = total_elapsed;
                        }
                    }
                    shift_in_progress = true;
                }
            } else {
                if (ratio_now < start_ratio-10) { // Downshift - Ratio should get larger so inverse means flaring
                    if (!this->flaring) {
                        dest_report.flare_timestamp = total_elapsed; // Mark report where flare occurred
                    }
                    this->flaring = true;
                } else if (ratio_now > start_ratio+10) {
                    if (!shift_in_progress) {
                        if (gen_report) {
                            dest_report.transition_start = total_elapsed;
                        }
                    }
                    shift_in_progress = true;
                }
            }
        }
        if (this->est_gear_idx == sd.targ_g) {
            if (dest_report.transition_end == 0) {
                dest_report.transition_end = total_elapsed;
            }
            egs_can_hal->set_torque_request(TorqueRequest::None);
            egs_can_hal->set_requested_torque(0);
        }

        if (gen_report && total_elapsed % SR_REPORT_INTERVAL == 0 && index < MAX_POINTS_PER_SR_ARRAY) {
            dest_report.engine_rpm[index] = (uint16_t)sensor_data.engine_rpm;
            dest_report.input_rpm[index] = (uint16_t)sensor_data.input_rpm;
            dest_report.output_rpm[index] = (uint16_t)sensor_data.output_rpm;
            dest_report.engine_torque[index] = (int16_t)sensor_data.static_torque;
            index++;
        }

        add_report = !add_report;
        pressure_mgr->set_target_spc_pressure(curr_spc_pressure);
        sd.shift_solenoid->write_pwm_12_bit(curr_sd_pwm); // Write to shift solenoid
        if (ss_open > 250) {
            curr_sd_pwm = 1200; // ~25% PWM (When adjusted for voltage) requested after 100ms to prevent SS from burning up
        }
        if (sd.shift_solenoid->get_pwm_compensated() != 0) {
            ss_open += SHIFT_DELAY_MS;
        }
        vTaskDelay(SHIFT_DELAY_MS);
        total_elapsed += SHIFT_DELAY_MS;
        stage_elapsed += SHIFT_DELAY_MS;
    }
    this->shift_stage = 0;
    this->is_ramp = false;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT", "Shift done");
    pressure_mgr->disable_spc(); // Max pressure!
    sd.shift_solenoid->write_pwm_12_bit(0); // Close SPC and Shift solenoid
    egs_can_hal->set_torque_request(TorqueRequest::None);
    egs_can_hal->set_requested_torque(0);
    this->tcc->on_shift_complete(sensor_data.current_timestamp_ms);
    if (gen_report) {
        // Copy the response len
        dest_report.report_array_len = index;
        dest_report.interval_points = SR_REPORT_INTERVAL;
        dest_report.total_ms = total_elapsed;
        dest_report.targ_curr = ((sd.targ_g & 0x0F) << 4) | (sd.curr_g & 0x0F);
        this->shift_reporter->add_report(dest_report);
    }
    if (this->pressure_mgr != nullptr) {
        this->pressure_mgr->perform_adaptation(&pre_shift, req_lookup, &dest_report, gen_report);
    };
    this->sensor_data.last_shift_time = esp_timer_get_time()/1000;
    this->flaring = false;
}

void Gearbox::shift_thread() {
    this->shifting = true;
    GearboxGear curr_target = this->target_gear;
    GearboxGear curr_actual = this->actual_gear;
    if (curr_actual == curr_target) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "SHIFTER", "Gears are the same????");
        goto cleanup;
    }
    if (!is_controllable_gear(curr_actual) && !is_controllable_gear(curr_target)) { // N->P or P->N
        this->pressure_mgr->set_target_spc_pressure(100);
        sol_y4->write_pwm_12_bit(800); // 3-4 is pulsed at 20%
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "No need to shift");
        this->actual_gear = curr_target; // Set on startup
        goto cleanup;
    } else if (is_controllable_gear(curr_actual) != is_controllable_gear(curr_target)) { // This would be a garage shift, either in or out
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Garage shift");
        if (is_controllable_gear(curr_target)) {
            // N/P -> R/D
            // Defaults (Start in 2nd)
            int spc_start = pressure_mgr->find_working_mpc_pressure(this->actual_gear, &sensor_data, gearboxConfig.max_torque);
            int spc_ramp = 10;
            uint16_t y4_pwm_val = 4096;
            pressure_mgr->set_target_mpc_pressure(pressure_mgr->find_working_mpc_pressure(this->actual_gear, &sensor_data, gearboxConfig.max_torque));
            pressure_mgr->set_target_spc_pressure(spc_start);
            sol_y4->write_pwm_12_bit(y4_pwm_val); // Full on
            vTaskDelay(150);
            uint16_t del = 0;
            while (spc_start <= pressure_mgr->find_working_mpc_pressure(this->actual_gear, &sensor_data, gearboxConfig.max_torque)*4) {
                spc_start += spc_ramp;
                pressure_mgr->set_target_mpc_pressure(pressure_mgr->find_working_mpc_pressure(this->actual_gear, &sensor_data, gearboxConfig.max_torque)*1.1);
                pressure_mgr->set_target_spc_pressure(spc_start);  
                if (del > 100) {
                    y4_pwm_val = 1024; // 25% on to prevent burnout
                }
                sol_y4->write_pwm_12_bit(y4_pwm_val); // Full on   
                del += 25;   
                vTaskDelay(6/portTICK_PERIOD_MS);
            }
            vTaskDelay(200);
            sol_y4->write_pwm_12_bit(0);
            pressure_mgr->disable_spc();      
        } else {
            // Garage shifting to N or P, we can just set the pressure back to idle
            pressure_mgr->set_target_spc_pressure(50);
            sol_y4->write_pwm_12_bit(800); // Back to idle
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
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Both gears are controllable");
        if (is_fwd_gear(curr_target) != is_fwd_gear(curr_actual)) {
            // In this case, we set the current gear to neutral, then thread will re-spawn
            ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Shifter got stuck in R-D. Returning and trying again");
            this->actual_gear = GearboxGear::Neutral;
            goto cleanup;
        } else if (is_fwd_gear(curr_target) && is_fwd_gear(curr_actual)){
            // Forward shift logic
            if (curr_target > curr_actual) { // Upshifting
                ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Upshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                //this->show_upshift = true;
                ProfileGearChange pgc;
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
                elapse_shift(pgc, prof, true);
                this->actual_gear = curr_target;
                this->start_second = true;
                goto cleanup;
            } else { // Downshifting
                ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Downshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                //this->show_downshift = true;
                ProfileGearChange pgc;
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
                elapse_shift(pgc, prof, false);
                this->actual_gear = curr_target;
                goto cleanup;
            }
        } else {
            ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Ignoring request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
        }
        goto cleanup;
    }
cleanup:
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Shift complete");
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
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "GEARBOX START!");
    uint64_t expire_check = esp_timer_get_time() + 100000; // 100ms
    while (esp_timer_get_time() < expire_check) {
        this->shifter_pos = egs_can_hal->get_shifter_position_ewm(esp_timer_get_time()/1000, 250);
        last_position = this->shifter_pos;
        if (this->shifter_pos == ShifterPosition::P || this->shifter_pos == ShifterPosition::N) {
            egs_can_hal->set_safe_start(true);
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
        if (this->diag_stop_control) {
            vTaskDelay(50);
            continue;
        }

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
            if (this->sensor_data.output_rpm > 0 && this->sensor_data.input_rpm > 0) {
                // Store our ratio
                this->sensor_data.gear_ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
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
        if (can_read && this->sensor_data.output_rpm >= 100) {
            bool rev = !is_fwd_gear(this->target_gear);
            if (!this->calcGearFromRatio(rev)) {
                //ESP_LOG_LEVEL(ESP_LOG_ERROR, "GEARBOX", "GEAR RATIO IMPLAUSIBLE");
            }
        }
        uint8_t p_tmp = egs_can_hal->get_pedal_value(now, 1000);
        if (p_tmp != 0xFF) {
            this->sensor_data.pedal_pos = p_tmp;
        }
        sensor_data.is_braking = egs_can_hal->get_is_brake_pressed(now, 1000);
        this->sensor_data.engine_rpm = egs_can_hal->get_engine_rpm(now, 1000);
        if (this->sensor_data.engine_rpm == UINT16_MAX) {
            this->sensor_data.engine_rpm = 0;
        } else if (asleep && egs_can_hal->get_terminal_15(now, 1000) == TerminalStatus::On) {
            egs_can_hal->enable_normal_msg_transmission();
            asleep = false; // Wake up!
        }
        // Update solenoids, only if engine RPM is OK
        if (this->sensor_data.engine_rpm > 500) {
            if (!shifting) { // If shifting then shift manager has control over MPC working
                this->mpc_working = pressure_mgr->find_working_mpc_pressure(this->actual_gear, &sensor_data, this->gearboxConfig.max_torque);
            }
            this->pressure_mgr->set_target_mpc_pressure(this->mpc_working);
        }
        if (Sensors::parking_lock_engaged(&lock_state)) {
            egs_can_hal->set_safe_start(lock_state);
            this->shifter_pos = egs_can_hal->get_shifter_position_ewm(now, 1000);
            if (
                this->shifter_pos == ShifterPosition::P ||
                this->shifter_pos == ShifterPosition::P_R ||
                this->shifter_pos == ShifterPosition::R ||
                this->shifter_pos == ShifterPosition::R_N ||
                this->shifter_pos == ShifterPosition::N ||
                this->shifter_pos == ShifterPosition::N_D ||
                this->shifter_pos == ShifterPosition::D ||
                this->shifter_pos == ShifterPosition::FOUR ||
                this->shifter_pos == ShifterPosition::THREE ||
                this->shifter_pos == ShifterPosition::TWO ||
                this->shifter_pos == ShifterPosition::ONE 
            ) {
                if (this->shifter_pos != last_position) {
                    if (lock_state) {
                        if (this->shifter_pos == ShifterPosition::P) {
                            this->target_gear = GearboxGear::Park;
                            last_position = this->shifter_pos;
                            if (this->pressure_mgr != nullptr) {
                                this->pressure_mgr->save();
                            }
                            this->shift_reporter->save();
                        } else if (this->shifter_pos == ShifterPosition::N) {
                            this->target_gear = GearboxGear::Neutral;
                            last_position = this->shifter_pos;
                        }
                    } else {
                        // Drive or R!
                        if (this->shifter_pos == ShifterPosition::R) {
                            this->target_gear = this->start_second ? GearboxGear::Reverse_Second : GearboxGear::Reverse_First;
                            last_position = this->shifter_pos;
                        } else if (this->shifter_pos == ShifterPosition::D || this->shifter_pos == ShifterPosition::N_D) {
                            // Some shift levers can be semi broken
                            // and can switch between ND and D randomly whilst in motion
                            // handle that case here by checking current fwd gear first
                            // If current gear is also fwd, ignore!
                            if (!is_fwd_gear(this->actual_gear) && !is_fwd_gear(this->target_gear)) {
                                this->target_gear = this->start_second ? GearboxGear::Second : GearboxGear::First;
                            }
                            last_position = this->shifter_pos;
                        }
                    }
                }
            }
        }
        if (this->sensor_data.engine_rpm > 500) {
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
                        if (calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, next, this->gearboxConfig.ratios) > MIN_WORKING_RPM+100) {
                            this->target_gear = next;
                        }
                    } else if ((this->ask_downshift || want_downshift) && this->actual_gear > GearboxGear::First) {
                        // Check RPMs
                        GearboxGear prev = prev_gear(this->actual_gear);
                        if (calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, prev, this->gearboxConfig.ratios) < this->redline_rpm-500) {
                            this->target_gear = prev;
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
                    if (is_fwd_gear(this->actual_gear) && is_fwd_gear(this->target_gear)) {
                        this->sensor_data.tcc_slip_rpm = sensor_data.engine_rpm - sensor_data.input_rpm;
                        if (this->tcc != nullptr) {
                            this->tcc->update(this->target_gear, this->pressure_mgr, this->current_profile, &this->sensor_data, this->shifting);
                            egs_can_hal->set_clutch_status(this->tcc->get_clutch_state());
                        }
                    } else {
                        this->tcc_percent = 0;
                        this->pressure_mgr->set_target_tcc_pressure(0);
                        egs_can_hal->set_clutch_status(ClutchStatus::Open);
                        //sol_tcc->write_pwm_12_bit(0);
                    }
                }
            }
            if (this->target_gear != this->actual_gear && this->shifting == false) {
                // Create shift task to change gears for us!
                xTaskCreatePinnedToCore(Gearbox::start_shift_thread, "Shift handler", 8192, this, 10, &this->shift_task, 1);
            }
        } else if (!shifting || asleep) {
            mpc_cc->set_target_current(0);
            spc_cc->set_target_current(0);
            sol_tcc->write_pwm_12_bit(0);
            sol_y3->write_pwm_12_bit(0);
            sol_y4->write_pwm_12_bit(0);
            sol_y5->write_pwm_12_bit(0);
        }
        int16_t tmp_atf = 0;
        if (lock_state || !Sensors::read_atf_temp(&tmp_atf)) {
            // Default to engine coolant
            tmp_atf = (egs_can_hal->get_engine_coolant_temp(now, 1000));
            if (tmp_atf != INT16_MAX) {
                this->sensor_data.atf_temp = tmp_atf;
            }
        } else {
            if (!temp_cal) {
                temp_cal = true;
                temp_at_test = tmp_atf;
                if (temp_at_test != 25) {
                    resistance_mpc = resistance_mpc + (resistance_mpc*(((25.0-(float)temp_at_test)*0.393)/100.0));
                    resistance_spc = resistance_spc + (resistance_spc*(((25.0-(float)temp_at_test)*0.393)/100.0));
                }
                ESP_LOGI("GB", "Calibrated solenoids at %d C. Adjusted for 25C: SPC %.2f MPC %.2f", tmp_atf, resistance_spc, resistance_mpc);
            }
            // SPC and MPC can cause voltage swing on the ATF line, so disable
            // monitoring when shifting gears!
            if (!shifting) {
                this->sensor_data.atf_temp = tmp_atf;
            }
        }

        // Check for vehicle in sleep mode
        if (!asleep && this->sensor_data.input_rpm == 0 && egs_can_hal->get_terminal_15(now, 1000) != TerminalStatus::On) {
            egs_can_hal->disable_normal_msg_transmission();
            // Feedback (Debug) - Car going to sleep
            spkr.send_note(3000, 100, 110);
            spkr.send_note(3000, 100, 110);
            spkr.send_note(3000, 100, 110);
            this->asleep = true;
        }

        egs_can_hal->set_gearbox_temperature(this->sensor_data.atf_temp);
        egs_can_hal->set_shifter_position(this->shifter_pos);
        egs_can_hal->set_input_shaft_speed(this->sensor_data.input_rpm);
        egs_can_hal->set_target_gear(this->target_gear);
        egs_can_hal->set_actual_gear(this->actual_gear);
        egs_can_hal->set_solenoid_pwm(sol_y3->get_pwm_compensated(), SolenoidName::Y3);
        egs_can_hal->set_solenoid_pwm(sol_y4->get_pwm_compensated(), SolenoidName::Y4);
        egs_can_hal->set_solenoid_pwm(sol_y5->get_pwm_compensated(), SolenoidName::Y5);
        egs_can_hal->set_solenoid_pwm(sol_spc->get_current(), SolenoidName::SPC);
        egs_can_hal->set_solenoid_pwm(sol_mpc->get_current(), SolenoidName::MPC);
        egs_can_hal->set_solenoid_pwm(sol_tcc->get_pwm_compensated(), SolenoidName::TCC);
        egs_can_hal->set_shift_stage(this->shift_stage, this->is_ramp);
        egs_can_hal->set_gear_ratio(this->sensor_data.gear_ratio*100);
        egs_can_hal->set_gear_disagree(this->gear_disagree_count);

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
        // Wheel torque
        if (this->sensor_data.gear_ratio == 0) {
            // Fallback ratio for when gear ratio is actually 0
            float f;
            switch (this->target_gear) {
                case GearboxGear::First:
                    f = gearboxConfig.ratios[0];
                    break;
                case GearboxGear::Second:
                    f = gearboxConfig.ratios[1];
                    break;
                case GearboxGear::Third:
                    f = gearboxConfig.ratios[2];
                    break;
                case GearboxGear::Fourth:
                    f = gearboxConfig.ratios[3];
                    break;
                case GearboxGear::Reverse_First:
                    f = gearboxConfig.ratios[4]*-1;
                    break;
                case GearboxGear::Reverse_Second:
                    f = gearboxConfig.ratios[4]*-1;
                    break;
                case GearboxGear::Park:
                case GearboxGear::SignalNotAvaliable:
                case GearboxGear::Neutral:
                default:
                    f = 0.0;
                    break;
            }
            egs_can_hal->set_wheel_torque_multi_factor(f);
        } else {
            egs_can_hal->set_wheel_torque_multi_factor(this->sensor_data.gear_ratio);
        }

        //ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Torque: MIN: %3d, MAX: %3d, STAT: %3d", min_torque, max_torque, static_torque);
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
            egs_can_hal->set_drive_profile(this->current_profile->get_profile());
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

bool Gearbox::calc_input_rpm(uint16_t* dest) {
    RpmReading rpm{};
    bool ok = false;
    bool conduct_sanity_check = gear_disagree_count == 0 && 
        (this->actual_gear == this->target_gear) && ( // Same gear (Not shifting)
        (this->actual_gear == GearboxGear::Second) || // And in 2..
        (this->actual_gear == GearboxGear::Third) || // .. or 3 ..
        (this->actual_gear == GearboxGear::Fourth) // .. or 4
    );
    if (Sensors::read_input_rpm(&rpm, conduct_sanity_check)) {
        ok = true;
        this->sensor_data.input_rpm = rpm.calc_rpm;
    }
    return ok;
}

bool Gearbox::calc_output_rpm(uint16_t* dest, uint64_t now) {
    WheelData left = egs_can_hal->get_rear_left_wheel(now, 500);
    WheelData right = egs_can_hal->get_rear_right_wheel(now, 500);
    //ESP_LOG_LEVEL(ESP_LOG_INFO, "WRPM","R:(%d %d) L:(%d %d)", (int)right.current_dir, right.double_rpm, (int)left.current_dir, left.double_rpm);
    float rpm = 0;
    if (left.current_dir == WheelDirection::SignalNotAvaliable && right.current_dir == WheelDirection::SignalNotAvaliable) {
        //ESP_LOG_LEVEL(ESP_LOG_ERROR, "CALC_OUTPUT_RPM", "Could not obtain right and left wheel RPM!");
        return false;
    } else if (left.current_dir == WheelDirection::SignalNotAvaliable) { // Right OK
        ESP_LOG_LEVEL(ESP_LOG_WARN, "CALC_OUTPUT_RPM", "Could not obtain left wheel RPM, trusting the right one!");
        rpm = right.double_rpm;
    } else if (right.current_dir == WheelDirection::SignalNotAvaliable) { // Left OK
        ESP_LOG_LEVEL(ESP_LOG_WARN, "CALC_OUTPUT_RPM", "Could not obtain right wheel RPM, trusting the left one!");
        rpm = left.double_rpm;
    } else { // Both sensors OK!
        rpm = (abs(left.double_rpm) + abs(right.double_rpm)) / 2;
    }
    rpm *= this->diff_ratio_f;
    rpm /= 2;
    *dest = rpm;
    return true;
}

bool Gearbox::calcGearFromRatio(bool is_reverse) {
    float ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
    if (is_reverse) {
        ratio *=-1;
        for(uint8_t i = 0; i < 2; i++) { // Scan the 2 reverse gears
            GearRatioLimit limits = gearboxConfig.bounds[i+5];
            if (ratio >= limits.min && ratio <= limits.max) {
                this->est_gear_idx = i+1;
                return true;
            }
        }
    } else {
        for(uint8_t i = 0; i < 5; i++) { // Scan the 5 forwards gears
            GearRatioLimit limits = gearboxConfig.bounds[i];
            if (ratio >= limits.min && ratio <= limits.max) {
                this->est_gear_idx = i+1;
                return true;
            }
        }
    }
    this->est_gear_idx = 0;
    return false;
}

Gearbox* gearbox = nullptr;