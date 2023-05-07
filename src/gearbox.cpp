#include "gearbox.h"
#include "nvs/eeprom_config.h"
#include "adv_opts.h"
#include <tcu_maths.h>
#include "solenoids/constant_current.h"
#include "speaker.h"
#include "esp_timer.h"

uint16_t Gearbox::redline_rpm = 4000;

#define max(a, b)           \
    ({                      \
        typeof(a) _a = (a); \
        typeof(b) _b = (b); \
        _a > _b ? _a : _b;  \
    })

// ONLY FOR FORWARD GEARS!
int calc_input_rpm_from_req_gear(int output_rpm, GearboxGear req_gear, GearboxConfiguration* gb_config)
{
    int calculated = output_rpm;
    switch (req_gear)
    {
    case GearboxGear::First:
        calculated *= gb_config->bounds[0].ratio;
        break;
    case GearboxGear::Second:
        calculated *= gb_config->bounds[1].ratio;
        break;
    case GearboxGear::Third:
        calculated *= gb_config->bounds[2].ratio;
        break;
    case GearboxGear::Fourth:
        calculated *= gb_config->bounds[3].ratio;
        break;
    case GearboxGear::Fifth:
        calculated *= gb_config->bounds[4].ratio;
        break;
    default:
        break;
    }
    return calculated;
}

Gearbox::Gearbox()
{
    this->current_profile = nullptr;
    egs_can_hal->set_drive_profile(GearboxProfile::Underscore); // Uninitialized
    this->profile_mutex = portMUX_INITIALIZER_UNLOCKED;
    this->sensor_data = SensorData{
        .input_rpm = 0,
        .engine_rpm = 0,
        .output_rpm = 0,
        .pedal_pos = 0,
        .atf_temp = 0,
        .static_torque = 0,
        .max_torque = 0,
        .min_torque = 0,
        .driver_requested_torque = 0,
        .tcc_slip_rpm = 0,
        .last_shift_time = 0,
        .current_timestamp_ms = (uint64_t)(esp_timer_get_time() / 1000),
        .is_braking = false,
        .d_output_rpm = 0,
        .gear_ratio = 0.0F,
        .rr_wheel = WheelData { .double_rpm = 0, .current_dir = WheelDirection::Stationary },
        .rl_wheel = WheelData { .double_rpm = 0, .current_dir = WheelDirection::Stationary },
        .fr_wheel = WheelData { .double_rpm = 0, .current_dir = WheelDirection::Stationary },
        .fl_wheel = WheelData { .double_rpm = 0, .current_dir = WheelDirection::Stationary },
    };
    this->output_data = OutputData {
        .torque_req_amount = 0,
        .torque_req_type = TorqueRequest::None
    };


    NAG_SETTINGS* nag_settings = VEHICLE_CONFIG.is_large_nag ? &NAG_CURRENT_SETTINGS.large_nag : &NAG_CURRENT_SETTINGS.small_nag;
    // Generate our ratio info
    this->gearboxConfig.max_torque = nag_settings->max_torque;
    this->gearboxConfig.bounds[0] = GearRatioInfo { // 1st 
        .ratio_max_drift = nag_settings->ratio_1*(1.0f+(float)(NAG_CURRENT_SETTINGS.max_drift_1/100.0f)),
        .ratio = nag_settings->ratio_1,
        .ratio_min_drift = nag_settings->ratio_1*(1.0f-(float)(NAG_CURRENT_SETTINGS.max_drift_1/100.0f)),
        .power_loss = (nag_settings->power_loss_1)/100.0f,
    };
    this->gearboxConfig.bounds[1] = GearRatioInfo { // 2nd 
        .ratio_max_drift = nag_settings->ratio_2*(1.0f+(float)(NAG_CURRENT_SETTINGS.max_drift_2/100.0f)),
        .ratio = nag_settings->ratio_2,
        .ratio_min_drift = nag_settings->ratio_2*(1.0f-(float)(NAG_CURRENT_SETTINGS.max_drift_2/100.0f)),
        .power_loss = (nag_settings->power_loss_2)/100.0f,
    };
    this->gearboxConfig.bounds[2] = GearRatioInfo { // 3rd 
        .ratio_max_drift = nag_settings->ratio_3*(1.0f+(float)(NAG_CURRENT_SETTINGS.max_drift_3/100.0f)),
        .ratio = nag_settings->ratio_3,
        .ratio_min_drift = nag_settings->ratio_3*(1.0f-(float)(NAG_CURRENT_SETTINGS.max_drift_3/100.0f)),
        .power_loss = (nag_settings->power_loss_3)/100.0f,
    };
    this->gearboxConfig.bounds[3] = GearRatioInfo { // 4th 
        .ratio_max_drift = nag_settings->ratio_4*(1.0f+(float)(NAG_CURRENT_SETTINGS.max_drift_4/100.0f)),
        .ratio = nag_settings->ratio_4,
        .ratio_min_drift = nag_settings->ratio_4*(1.0f-(float)(NAG_CURRENT_SETTINGS.max_drift_4/100.0f)),
        .power_loss = (nag_settings->power_loss_4)/100.0f,
    };
    this->gearboxConfig.bounds[4] = GearRatioInfo { // 5th 
        .ratio_max_drift = nag_settings->ratio_5*(1.0f+(float)(NAG_CURRENT_SETTINGS.max_drift_5/100.0f)),
        .ratio = nag_settings->ratio_5,
        .ratio_min_drift = nag_settings->ratio_5*(1.0f-(float)(NAG_CURRENT_SETTINGS.max_drift_5/100.0f)),
        .power_loss = (nag_settings->power_loss_5)/100.0f,
    };
    this->gearboxConfig.bounds[5] = GearRatioInfo { // r1 
        .ratio_max_drift = nag_settings->ratio_r1*(1.0f+(float)(NAG_CURRENT_SETTINGS.max_drift_r1/100.0f)),
        .ratio = nag_settings->ratio_r1,
        .ratio_min_drift = nag_settings->ratio_r1*(1.0f-(float)(NAG_CURRENT_SETTINGS.max_drift_r1/100.0f)),
        .power_loss = (nag_settings->power_loss_r1)/100.0f,
    };
    this->gearboxConfig.bounds[6] = GearRatioInfo { // r2 
        .ratio_max_drift = nag_settings->ratio_r2*(1.0f+(float)(NAG_CURRENT_SETTINGS.max_drift_r2/100.0f)),
        .ratio = nag_settings->ratio_r2,
        .ratio_min_drift = nag_settings->ratio_r2*(1.0f-(float)(NAG_CURRENT_SETTINGS.max_drift_r2/100.0f)),
        .power_loss = (nag_settings->power_loss_r2)/100.0f,
    };

    this->pressure_mgr = new PressureManager(&this->sensor_data, this->gearboxConfig.max_torque);
    this->tcc = new TorqueConverter(this->gearboxConfig.max_torque);
    this->shift_reporter = new ShiftReporter();
    this->itm = new InputTorqueModel();
    pressure_manager = this->pressure_mgr;
    // Wait for solenoid routine to complete
    if (!Solenoids::init_routine_completed())
    {
        vTaskDelay(1);
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "---GEARBOX INFO---");
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Max torque: %d Nm", this->gearboxConfig.max_torque);
    for (int i = 0; i < 7; i++)
    {
        char c = i < 5 ? 'D' : 'R';
        int g = i < 5 ? i + 1 : i - 4;
        ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Gear ratio %c%d %.3f. Bounds: (%.3f - %.3f). Power loss %.2f", c, g, gearboxConfig.bounds[i].ratio, gearboxConfig.bounds[i].ratio_max_drift, gearboxConfig.bounds[i].ratio_min_drift, gearboxConfig.bounds[i].power_loss);
    }
    if (VEHICLE_CONFIG.engine_type == 1)
    {
        this->redline_rpm = VEHICLE_CONFIG.red_line_rpm_petrol;
    }
    else
    {
        this->redline_rpm = VEHICLE_CONFIG.red_line_rpm_diesel;
    }
    if (this->redline_rpm < 4000)
    {
        this->redline_rpm = 4000; // just in case
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "---OTHER CONFIG---");
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Redline RPM: %d", this->redline_rpm);
    this->diff_ratio_f = (float)VEHICLE_CONFIG.diff_ratio / 1000.0;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Diff ratio: %.2f", this->diff_ratio_f);
}

void Gearbox::set_profile(AbstractProfile *prof)
{
    if (prof != nullptr)
    { // Only change if not nullptr!
        portENTER_CRITICAL(&this->profile_mutex);
        this->current_profile = prof;
        portEXIT_CRITICAL(&this->profile_mutex);
    }
}

esp_err_t Gearbox::start_controller()
{
    xTaskCreatePinnedToCore(Gearbox::start_controller_internal, "GEARBOX", 32768, static_cast<void *>(this), 10, nullptr, 1);
    return ESP_OK;
}

GearboxGear gear_from_idx(uint8_t idx)
{
    // Only for drivable gears. P/R/SNV is never used
    GearboxGear ret = GearboxGear::SignalNotAvailable;
    if (likely(idx >= 1 && idx <= 5)) {
        ret = (GearboxGear)(idx); // Direct cast for gears 1-5
    } else if (idx == 6) {
        ret = GearboxGear::Reverse_First;
    } else if (idx == 7) {
        ret = GearboxGear::Reverse_Second;
    }
    return ret;
}

bool is_controllable_gear(GearboxGear g)
{
    bool controllable = true;
    if (unlikely(g == GearboxGear::Park || g == GearboxGear::Neutral || g == GearboxGear::SignalNotAvailable)) {
        controllable = false;
    }
    return controllable;
}

bool is_fwd_gear(GearboxGear g)
{
    bool is_fwd = false;
    if (likely((uint8_t)g >= 1 && (uint8_t)g <= 5)) {
        is_fwd = true;
    }
    return is_fwd;
}

const char *gear_to_text(GearboxGear g)
{
    switch (g)
    {
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
    case GearboxGear::Reverse_First:
        return "R1";
    case GearboxGear::Reverse_Second:
        return "R2";
    case GearboxGear::Park:
        return "P";
    case GearboxGear::SignalNotAvailable:
        return "SNA";
    case GearboxGear::Neutral:
        return "N";
    default:
        return "";
    }
}

void Gearbox::inc_gear_request()
{
    this->ask_upshift = true;
    this->ask_downshift = false;
}

void Gearbox::dec_gear_request()
{
    this->ask_upshift = false;
    this->ask_downshift = true;
}

void Gearbox::set_torque_request(TorqueRequest type, float amount) {
    this->output_data.torque_req_amount = amount;
    this->output_data.torque_req_type = type;
    egs_can_hal->set_torque_request(type, amount);
}

GearboxGear next_gear(GearboxGear g)
{
    GearboxGear next = g;
    uint8_t idx = (uint8_t)g;
    if (idx >= 1 && idx < 5) { // 1-4
        next = (GearboxGear)(idx+1);
    }
    return next;
}

GearboxGear prev_gear(GearboxGear g)
{
    GearboxGear prev = g;
    uint8_t idx = (uint8_t)g;
    if (idx > 1 && idx <= 5) { // 2-5
        prev = (GearboxGear)(idx-1);
    }
    return prev;
}

#define SHIFT_DELAY_MS 10     // 10ms steps
#define MIN_RATIO_CALC_RPM 200 // Min INPUT RPM for ratio calculations and RPM readings

/**
 * @brief Linear interpolate between 2 values
 *
 * @param start_value Start value (At time 0)
 * @param end_value End target value (At `interp_duration` ms from start)
 * @param current_elapsed The current elapsed time
 * @param interp_duration Total duration for the interpolation
 * @return int16_t interpolated current value
 */
float linear_interp(float start_value, float end_value, uint16_t current_elapsed, uint16_t interp_duration) {
    float ret;
    if (current_elapsed >= interp_duration) {
        ret = end_value;
    } else if (current_elapsed == 0) {
        ret = start_value;
    } else if (start_value == end_value) {
        ret = start_value; // Same, no change
    } else {
        float step_size = (end_value-start_value) / (float)(interp_duration);
        ret = start_value + (step_size*current_elapsed);
    }
    return ret;
}

ShiftReportSegment Gearbox::collect_report_segment(uint64_t start_time) {
    return ShiftReportSegment {
        .static_torque = sensor_data.static_torque,
        .driver_torque = sensor_data.driver_requested_torque,
        .egs_req_torque = (int16_t)((this->output_data.torque_req_type == TorqueRequest::None) ? INT16_MAX : (int16_t)(this->output_data.torque_req_amount)),
        .engine_rpm = sensor_data.engine_rpm,
        .input_rpm = sensor_data.input_rpm,
        .output_rpm = sensor_data.output_rpm,
        .mpc_pressure = this->pressure_mgr->get_targ_mpc_pressure(),
        .spc_pressure = this->pressure_mgr->get_targ_spc_pressure(),
        .timestamp = (uint16_t)(sensor_data.current_timestamp_ms-start_time)
    };
}

/**
 * @brief Used to shift between forward gears
 *
 * @return uint16_t - The actual time taken to shift gears. This is fed back into the adaptation network so it can better meet 'target_shift_duration_ms'
 */
bool Gearbox::elapse_shift(ProfileGearChange req_lookup, AbstractProfile *profile, bool is_upshift)
{
    bool result = false;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ELAPSE_SHIFT", "Shift started!");
    if (nullptr != profile)
    {
        ShiftReport sr = ShiftReport{};
        sr.profile = profile->get_profile_id();
        sr.change = req_lookup;
        sr.atf_temp_c = sensor_data.atf_temp;
        uint64_t shift_start_time = sensor_data.current_timestamp_ms;
        uint8_t overlap_report_size = 0;
        ShiftCharacteristics chars = profile->get_shift_characteristics(req_lookup, &this->sensor_data);
        ShiftData sd = pressure_mgr->get_shift_data(&this->gearboxConfig, req_lookup, chars, this->mpc_working);
        if (this->last_shift_solenoid == sd.shift_solenoid) { // Same shift solenoid
            while (sensor_data.current_timestamp_ms - sensor_data.last_shift_time < 1000) {
                vTaskDelay(10);
            }
            // Recalculate
            sd = pressure_mgr->get_shift_data(&this->gearboxConfig, req_lookup, chars, this->mpc_working);
            sd.bleed_data.spc_pressure = sd.bleed_data.spc_pressure/2; // Even lower pressure in bleed phase
        }
        this->last_shift_solenoid = sd.shift_solenoid;

        ShiftPhase* current_phase_data = &sd.bleed_data;
        uint8_t current_phase = SHIFT_PHASE_BLEED;
        Clutch apply_clutch = pressure_manager->get_clutch_to_apply(req_lookup);
        Clutch release_clutch = pressure_manager->get_clutch_to_release(req_lookup);
        // Needed for Bleed phase
        uint32_t phase_elapsed = 0u;
        uint32_t sol_open_time = 0u;
        // Bleed phase is phase 1, set the times here
        uint32_t phase_hold_time = sd.bleed_data.hold_time;
        uint32_t phase_ramp_time = sd.bleed_data.ramp_time;
        float max_spc = 0;
        float prev_phase_mpc;
        float curr_phase_mpc;
        float current_mpc;
        float prev_phase_spc;
        float curr_phase_spc;
        float current_spc;
        float prev_phase_delta_mpc;
        float curr_phase_delta_mpc;
        float current_delta_mpc;
        float prev_phase_delta_spc;
        float curr_phase_delta_spc;
        float current_delta_spc;
        prev_phase_mpc = curr_phase_mpc = current_mpc = this->mpc_working;
        prev_phase_spc = curr_phase_spc = current_spc = 50.0F;
        prev_phase_delta_mpc = curr_phase_delta_mpc = current_delta_mpc = 0.0F;
        prev_phase_delta_spc = curr_phase_delta_spc = current_delta_spc = 0.0F;

        bool process_shift = true;
        float mpc_hold_adder = 0.0F;
        sr.start_reading = this->collect_report_segment(shift_start_time);
        float curr_torq_request = 0;
        float mpc_release_delay = 0;
        float d_trq = 0;
        float clamped_trq = 0;
        while(process_shift) {
            int rpm_target_gear = calc_input_rpm_from_req_gear(sensor_data.output_rpm, this->target_gear, &this->gearboxConfig);
            int rpm_current_gear = calc_input_rpm_from_req_gear(sensor_data.output_rpm, this->actual_gear, &this->gearboxConfig);
            // int rpm_to_target_gear = abs(sensor_data.input_rpm - rpm_current_gear);
            int current_trq = sensor_data.input_torque;
            if (sensor_data.static_torque > 0) {
                if (is_upshift && SBS_CURRENT_SETTINGS.upshift_use_driver_torque_as_input) {
                    current_trq = sensor_data.driver_requested_torque;
                } else if (!is_upshift && SBS_CURRENT_SETTINGS.downshift_use_driver_torque_as_input) {
                    current_trq = sensor_data.driver_requested_torque;
                }
            }

            if (phase_elapsed > phase_hold_time+phase_ramp_time && current_phase < SHIFT_PHASE_OVERLAP) {
                phase_elapsed = 0;
                current_phase++;
                // Set pressures from the previous phase
                prev_phase_mpc = current_mpc;
                prev_phase_spc = current_spc;
                prev_phase_delta_mpc = current_delta_mpc;
                prev_phase_delta_spc = current_delta_spc;

                // Set data for the next phase
                if (SHIFT_PHASE_BLEED == current_phase) {
                    current_phase_data = &sd.bleed_data;
                } else if (SHIFT_PHASE_FILL == current_phase) {
                    sr.bleed_reading = this->collect_report_segment(shift_start_time); // End of bleed
                    current_phase_data = &sd.fill_data;
                    // Open the shift solenoid on starting this phase!
                    sd.shift_solenoid->write_pwm_12_bit(4096);
                    sol_open_time = sensor_data.current_timestamp_ms;
                } else if (SHIFT_PHASE_TORQUE == current_phase) {
                    sr.prefill_reading = this->collect_report_segment(shift_start_time); // End of bleed
                    pressure_mgr->make_torque_and_overlap_data(&sd.torque_data, &sd.overlap_data, &sd.fill_data, chars, req_lookup, this->mpc_working);
                    current_phase = SHIFT_PHASE_OVERLAP; // Bypass
                    current_phase_data = &sd.overlap_data;
                } else if (SHIFT_PHASE_OVERLAP == current_phase) {
                    current_phase_data = &sd.overlap_data;
                    // Check if we are coasting or not
                }
                // Time targets are always static
                phase_hold_time = current_phase_data->hold_time;
                phase_ramp_time = current_phase_data->ramp_time;
                // Set pressure values for our new phase
                if (current_phase_data->mpc_offset_mode) {
                    curr_phase_mpc = 0;
                    curr_phase_delta_mpc = current_phase_data->mpc_pressure;
                } else {
                    curr_phase_mpc = current_phase_data->mpc_pressure;
                    curr_phase_delta_mpc = 0;
                }
                if (current_phase_data->spc_offset_mode) {
                    curr_phase_spc = 0;
                    curr_phase_delta_spc = current_phase_data->spc_pressure;
                } else {
                    curr_phase_spc = current_phase_data->spc_pressure;
                    curr_phase_delta_spc = 0;
                }
            }

            // TODO Transition this based on gear ratio!
            uint16_t now_working_mpc = pressure_manager->find_working_mpc_pressure(this->actual_gear);
            if (current_phase_data->mpc_offset_mode) { // Offset mode! So MPC for this phase should be current MPC working
                curr_phase_mpc = now_working_mpc;
            }
            if (SHIFT_PHASE_TORQUE == current_phase) {

            } else if (SHIFT_PHASE_OVERLAP == current_phase) {
                int min_spc;
                float spc_trq_multi;
                ShiftCharacteristics ss_now = profile->get_shift_characteristics(req_lookup, &this->sensor_data);
                // 0% - 100% (Or more)
                float torque_decimal = (float)(abs(current_trq)*100.0) / (float)(gearboxConfig.max_torque);
                switch (req_lookup) {
                    case ProfileGearChange::ONE_TWO: // K1
                        min_spc = 50;
                        spc_trq_multi = scale_number(ss_now.target_shift_time, 20.0, 4.00, 100, 1000);
                        break;
                    case ProfileGearChange::TWO_THREE: // K2
                        min_spc = 200;
                        spc_trq_multi = scale_number(ss_now.target_shift_time, 30.0, 7, 100, 1000);
                        break;
                    case ProfileGearChange::THREE_FOUR: // K3
                        min_spc = 750;
                        spc_trq_multi = scale_number(ss_now.target_shift_time, 50, 15, 100, 1000);
                        break;
                    case ProfileGearChange::FOUR_FIVE: // B1
                        min_spc = 100;
                        spc_trq_multi = scale_number(ss_now.target_shift_time, 29.0, 7, 100, 1000);
                        break;
                    case ProfileGearChange::FIVE_FOUR: // K1
                        spc_trq_multi = scale_number(ss_now.target_shift_time, 15, 6.5, 100, 1000);
                        min_spc = 100;
                        break;
                    case ProfileGearChange::FOUR_THREE: // B2
                        min_spc = 400;
                        spc_trq_multi = scale_number(ss_now.target_shift_time, 18.0, 9, 100, 1000);
                        break;
                    case ProfileGearChange::THREE_TWO: // K3
                        min_spc = 750;
                        spc_trq_multi = scale_number(ss_now.target_shift_time, 50, 15, 100, 1000);
                        break;
                    case ProfileGearChange::TWO_ONE: // B1
                    default:
                        min_spc = 50;
                        spc_trq_multi = scale_number(ss_now.target_shift_time, 11.5, 4.35, 100, 1000);
                        break;
                }
                if (sensor_data.static_torque < 0) {
                    spc_trq_multi *= 2; // For coast shifting
                }
                curr_phase_mpc = MAX(curr_phase_spc, now_working_mpc);
                curr_phase_delta_spc = MAX(max_spc, MAX(min_spc, torque_decimal*spc_trq_multi));
                if (phase_elapsed > phase_hold_time+phase_ramp_time && sd.targ_g != this->est_gear_idx) {
                    // Not shifting, try to increase SPC a bit more!
                    max_spc += scale_number(ss_now.target_shift_time, 50, 5, 100, 1000);
                }
                if (phase_elapsed % 250 == 0 && overlap_report_size < MAX_OVERLAP_REPORTS) {
                    sr.overlap_readings[overlap_report_size] = this->collect_report_segment(shift_start_time);
                    overlap_report_size += 1;
                }
            }

            // Interpolation
            // Values are clipped to end value after ramp time
            // For both MPC and SPC we want the following:
            // 1. Slowly interpolate the delta pressure from the old value to new value
            // 2. Slowly interpolate the realtime pressure from the old value to new value
            // 3. Combine the 2
            current_mpc = linear_interp(prev_phase_mpc, curr_phase_mpc, phase_elapsed, phase_ramp_time);
            if (current_phase_data->spc_offset_mode) {
                curr_phase_spc = curr_phase_mpc;
            }
            current_spc = linear_interp(prev_phase_spc, curr_phase_spc, phase_elapsed, phase_ramp_time);
            current_delta_mpc = linear_interp(prev_phase_delta_mpc, curr_phase_delta_mpc, phase_elapsed, phase_ramp_time);
            current_delta_spc = linear_interp(prev_phase_delta_spc, curr_phase_delta_spc, phase_elapsed, phase_ramp_time);
            // Set pressures and solenoid actuation
            float spc = current_spc + current_delta_spc;
            if (current_phase > SHIFT_PHASE_FILL) {
                spc =  MAX(sd.fill_data.spc_pressure, current_spc + current_delta_spc);
            }
            if (current_phase < SHIFT_PHASE_OVERLAP) {
                // Prevent MPC from being too low in bleed and fill phase
                mpc_hold_adder = pressure_manager->get_mpc_hold_adder(apply_clutch);
                this->mpc_working = MAX(MAX(current_mpc + current_delta_mpc, spc + SBS_CURRENT_SETTINGS.min_spc_delta_mpc), now_working_mpc + mpc_hold_adder);
            } else if (current_phase == SHIFT_PHASE_OVERLAP) {
                // Overlap
                float x = linear_interp(mpc_hold_adder, 0, phase_elapsed, phase_ramp_time+(mpc_release_delay*phase_hold_time));
                this->mpc_working = ((current_mpc + current_delta_mpc) + x);
            } else {
                this->mpc_working = current_mpc + current_delta_mpc;
            }
            pressure_mgr->set_target_spc_pressure(spc);

            if (SBS_CURRENT_SETTINGS.shift_solenoid_pwm_reduction_time < sensor_data.current_timestamp_ms - sol_open_time) {
                sd.shift_solenoid->write_pwm_12_bit(1200); // 33% to prevent burnout
            }

            bool coasting_shift = 0 > sensor_data.static_torque;
            if (SHIFT_PHASE_OVERLAP == current_phase && phase_elapsed >= phase_hold_time+phase_ramp_time) { // Check for completion! (Separate if block for simplicity)
                // MPC should be slightly higher than normal
                if (MIN_RATIO_CALC_RPM < sensor_data.input_rpm && this->est_gear_idx == sd.targ_g) { // Confirmed shift!
                    result = true;
                    process_shift = false;
                    sr.detect_shift_end_ts = (uint16_t)(sensor_data.current_timestamp_ms - shift_start_time);
                } else if (MIN_RATIO_CALC_RPM > sensor_data.input_rpm && phase_elapsed > 1000) {
                    result = true;
                    process_shift = false;
                } else if (!coasting_shift && MAX(SBS_CURRENT_SETTINGS.shift_timeout_pulling, (phase_hold_time+phase_ramp_time)*2) < phase_elapsed) { // TIMEOUT
                    result = false;
                    process_shift = false;
                } else if (coasting_shift && MAX(SBS_CURRENT_SETTINGS.shift_timeout_coasting, (phase_hold_time+phase_ramp_time)*2) < phase_elapsed) { // TIMEOUT
                    result = false;
                    process_shift = false;
                }
            }

            if (MIN_RATIO_CALC_RPM < sensor_data.input_rpm) {
                int shift_progress_percentage =  progress_between_targets(sensor_data.input_rpm, rpm_current_gear, rpm_target_gear);
                // Do ratio comparison to see if we are flaring
                // Both have 2 cases.
                //
                // Case 1. Engine is being pulled. If the gearbox flares here, the engine RPM
                //         will drop towards idle RPM
                // Case 2. Engine is pushing. If the gearbox flares here, the engine RPM
                //         will increase quickly
                //
                // To protect the gearbox in either case, it is best to ask the engine to work with the box rather than against it
                // by doing torque requests, rather than just increase SPC pressure, as that would cause a harsh shift to the user
                if (is_upshift) { // Upshift
                    if (sensor_data.input_rpm > rpm_current_gear + SBS_CURRENT_SETTINGS.delta_rpm_flare_detect) { // Flaring, rpm increase
                        flaring = true;
                    } else if (sensor_data.input_rpm < rpm_target_gear - SBS_CURRENT_SETTINGS.delta_rpm_flare_detect) { // RPM drop on shift end
                        flaring = true;
                    } else {
                        flaring = false;
                    }
                } else { // Downshift
                    if (sensor_data.input_rpm > rpm_target_gear + SBS_CURRENT_SETTINGS.delta_rpm_flare_detect) { // RPM overshoot!
                        flaring = true;
                    } else if (sensor_data.input_rpm < rpm_current_gear - SBS_CURRENT_SETTINGS.delta_rpm_flare_detect) { // RPM drop on shift start
                        flaring = true;
                    } else {
                        flaring = false;
                    }
                }
                if (flaring && sr.detect_flare_ts == 0) {
                    sr.detect_flare_ts = (uint16_t)(sensor_data.current_timestamp_ms - shift_start_time);
                }
                if (!flaring && shift_progress_percentage > 2 && sr.detect_shift_start_ts == 0) {
                    sr.detect_shift_start_ts = (uint16_t)(sensor_data.current_timestamp_ms - shift_start_time);
                }
                // Now check if we are in gear! If we are, we can exit!
                if (this->est_gear_idx == sd.targ_g) {
                    result = true;
                    process_shift = false;
                    sr.detect_shift_end_ts = (uint16_t)(sensor_data.current_timestamp_ms - shift_start_time);
                    break;
                }

                // Torque request behaviour (Experiment for EGS52 and ME2.7/8)
                if (current_phase >= SHIFT_PHASE_BLEED && ((SBS_CURRENT_SETTINGS.torque_request_upshift && is_upshift) || (SBS_CURRENT_SETTINGS.torque_request_downshift && !is_upshift))) {
                    if (d_trq == 0) {
                        int t_now = MAX(sensor_data.static_torque, sensor_data.driver_requested_torque);
                        // Multi increases with output torque. 25% of 100Nm is 25Nm whilst 10% of 580Nm is still 58Nm reduction!
                        float multi = scale_number(t_now, &SBS_CURRENT_SETTINGS.torque_reduction_factor_input_torque);
                        multi *= scale_number(chars.target_shift_time, &SBS_CURRENT_SETTINGS.torque_reduction_factor_shift_speed);
                        d_trq = (t_now * multi); // Our offset from pedal torque (Max)
                    }
                    int shift_progress_clamped = MIN(MAX(shift_progress_percentage, 0), 100);
                    TorqueRequest req = TorqueRequest::LessThan;
                    // start reduction
                    if (shift_progress_clamped < SBS_CURRENT_SETTINGS.torque_request_downramp_percent) { // Decrease torque from driver demand
                        curr_torq_request = MAX(0, linear_interp(MAX(sensor_data.driver_requested_torque, sensor_data.static_torque), sensor_data.driver_requested_torque - d_trq , shift_progress_clamped, SBS_CURRENT_SETTINGS.torque_request_downramp_percent));
                    } else if (shift_progress_clamped < SBS_CURRENT_SETTINGS.torque_request_hold_percent) { // Hold phase
                        curr_torq_request = MAX(0, sensor_data.driver_requested_torque - d_trq);
                    } else { // Nearing the end 75%+
                        req = TorqueRequest::LessThanFast;
                        curr_torq_request = linear_interp(MAX(0, sensor_data.driver_requested_torque-d_trq), sensor_data.driver_requested_torque, shift_progress_clamped-SBS_CURRENT_SETTINGS.torque_request_hold_percent, SBS_CURRENT_SETTINGS.torque_request_hold_percent);
                    }
                    this->set_torque_request(req, curr_torq_request);
                }
            } else {
                this->flaring = false;
                // No torque request if stationary
                this->set_torque_request(TorqueRequest::None, 0);
            }
            vTaskDelay(SHIFT_DELAY_MS / portTICK_PERIOD_MS);
            phase_elapsed += SHIFT_DELAY_MS;
        }

        // Only do max pressure phase if we shifted
        if (result) {
            ESP_LOGI("SHIFT","Starting max lock phase");
            float start_spc = current_spc + current_delta_spc;
            int old_spc = current_spc + current_delta_spc;
            uint16_t e = 0;
            while (e < sd.max_pressure_data.hold_time + sd.max_pressure_data.ramp_time) {
                this->mpc_working = pressure_manager->find_working_mpc_pressure(this->target_gear);
                float c = linear_interp(start_spc, MAX(this->mpc_working*2.5, old_spc*2), e, sd.max_pressure_data.ramp_time);
                pressure_manager->set_target_spc_pressure(c);
                this->mpc_working = pressure_manager->find_working_mpc_pressure(this->target_gear);
                // Finish torque request
                if (curr_torq_request != 0 && curr_torq_request < sensor_data.driver_requested_torque) {
                    // End target is always pedal torque
                    this->set_torque_request(TorqueRequest::LessThanFast, linear_interp(curr_torq_request, sensor_data.driver_requested_torque, e, sd.max_pressure_data.ramp_time+sd.max_pressure_data.hold_time));
                } else {
                    this->set_torque_request(TorqueRequest::None, 0);
                }
                vTaskDelay(20);
                e += 20;
            }
        }
        sr.end_reading = this->collect_report_segment(shift_start_time);
        sr.overlap_reading_size = overlap_report_size;
        sr.shift_status = result;
        sr.target_shift_speed = chars.target_shift_time;
        //egs_can_hal->set_fake_engine_rpm(0);
        pressure_manager->disable_spc();
        sd.shift_solenoid->write_pwm_12_bit(0);
        this->set_torque_request(TorqueRequest::None, 0);
        this->abort_shift = false;
        this->sensor_data.last_shift_time = sensor_data.current_timestamp_ms;
        this->flaring = false;
        if (result) { // Only set gear on conformation!
            ESP_LOGI("SHIFT", "SHIFT OK!");
            this->actual_gear = gear_from_idx(sd.targ_g);
        } else {
            ESP_LOGE("SHIFT", "Shift failed! End ratio is %.2f", (float)sensor_data.gear_ratio/100.0);
            this->target_gear = this->actual_gear;
        }
    }
    return result;
}

void Gearbox::shift_thread()
{
    this->shifting = true;
    GearboxGear curr_target = this->target_gear;
    GearboxGear curr_actual = this->actual_gear;
    if (curr_actual == curr_target)
    {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "SHIFTER", "Gears are the same????");
        goto cleanup;
    }
    if (!is_controllable_gear(curr_actual) && !is_controllable_gear(curr_target))
    { // N->P or P->N
        this->pressure_mgr->set_target_spc_pressure(this->mpc_working+100);
        sol_y4->write_pwm_12_bit(800); // 3-4 is pulsed at 20%
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "No need to shift");
        this->actual_gear = curr_target; // Set on startup
        goto cleanup;
    }
    else if (is_controllable_gear(curr_actual) != is_controllable_gear(curr_target))
    { // This would be a garage shift, either in or out
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Garage shift");
        bool activate_y3 = false;
        if (is_controllable_gear(curr_target))
        {
            // N/P -> R/D
            // Defaults (Start in 2nd)
            egs_can_hal->set_garage_shift_state(true);
            uint16_t spc = 600;
            activate_y3 = is_fwd_gear(curr_target) && !start_second && (last_fwd_gear == GearboxGear::First ||last_fwd_gear == GearboxGear::Second);
            pressure_mgr->set_target_spc_pressure(spc);
            vTaskDelay(100);
            if (activate_y3) { sol_y3->write_pwm_12_bit(4096); }
            sol_y4->write_pwm_12_bit(4096);
            uint16_t elapsed = 0;
            this->mpc_working = 1000;
            while (elapsed <= 250)
            {
                pressure_mgr->set_target_spc_pressure(spc);
                elapsed += 20;
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            if (activate_y3) { sol_y3->write_pwm_12_bit(1024); }
            sol_y4->write_pwm_12_bit(1024);
            while (spc <= this->mpc_working) {
                pressure_mgr->set_target_spc_pressure(spc);
                spc+=10;
                vTaskDelay(10);
            }
            vTaskDelay(250);
            sol_y4->write_pwm_12_bit(0);
            if (activate_y3) { sol_y3->write_pwm_12_bit(0); }
            pressure_mgr->disable_spc();
            egs_can_hal->set_garage_shift_state(false);
        }
        else
        {
            // Garage shifting to N or P, we can just set the pressure back to idle
            pressure_mgr->set_target_spc_pressure(pressure_manager->find_working_mpc_pressure(GearboxGear::Neutral)*1.5);
            sol_y4->write_pwm_12_bit(1024); // Back to idle
        }
        if (is_fwd_gear(curr_target))
        {
            // Last forward known gear
            // This way we better handle shifting from N->D at speed!
            this->actual_gear = this->last_fwd_gear;
            if (activate_y3) {
                this->actual_gear = GearboxGear::First;
            }
        }
        else
        {
            this->actual_gear = curr_target; // R1/R2
        }
        goto cleanup;
    }
    else
    { // Both gears are controllable
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Both gears are controllable");
        if (is_fwd_gear(curr_target) != is_fwd_gear(curr_actual))
        {
            // In this case, we set the current gear to neutral, then thread will re-spawn
            ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Shifter got stuck in R-D. Returning and trying again");
            this->actual_gear = GearboxGear::Neutral;
            goto cleanup;
        }
        else if (is_fwd_gear(curr_target) && is_fwd_gear(curr_actual))
        {
            // Forward shift logic
            if (curr_target > curr_actual)
            { // Upshifting
                ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Upshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                // this->show_upshift = true;
                ProfileGearChange pgc;
                if (curr_target == GearboxGear::Second)
                { // 1-2
                    pgc = ProfileGearChange::ONE_TWO;
                }
                else if (curr_target == GearboxGear::Third)
                { // 2-3
                    pgc = ProfileGearChange::TWO_THREE;
                }
                else if (curr_target == GearboxGear::Fourth)
                { // 3-4
                    pgc = ProfileGearChange::THREE_FOUR;
                }
                else if (curr_target == GearboxGear::Fifth)
                { // 4-5
                    pgc = ProfileGearChange::FOUR_FIVE;
                }
                else
                { // WTF
                    this->target_gear = this->actual_gear;
                    goto cleanup;
                }
                this->shift_idx = pgc;
                portENTER_CRITICAL(&this->profile_mutex);
                AbstractProfile *prof = this->current_profile;
                portEXIT_CRITICAL(&this->profile_mutex);
                elapse_shift(pgc, prof, true);
                this->start_second = true;
                goto cleanup;
            }
            else
            { // Downshifting
                ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Downshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                // this->show_downshift = true;
                ProfileGearChange pgc;
                if (curr_target == GearboxGear::First)
                { // 2-1
                    pgc = ProfileGearChange::TWO_ONE;
                    this->start_second = false;
                }
                else if (curr_target == GearboxGear::Second)
                { // 3-2
                    pgc = ProfileGearChange::THREE_TWO;
                    this->start_second = true;
                }
                else if (curr_target == GearboxGear::Third)
                { // 4-3
                    pgc = ProfileGearChange::FOUR_THREE;
                }
                else if (curr_target == GearboxGear::Fourth)
                { // 5-4
                    pgc = ProfileGearChange::FIVE_FOUR;
                }
                else
                { // WTF
                    this->target_gear = this->actual_gear;
                    goto cleanup;
                }
                this->shift_idx = pgc;
                portENTER_CRITICAL(&this->profile_mutex);
                AbstractProfile *prof = this->current_profile;
                portEXIT_CRITICAL(&this->profile_mutex);
                elapse_shift(pgc, prof, false);
                goto cleanup;
            }
        }
        else
        {
            ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Ignoring request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
        }
        goto cleanup;
    }
cleanup:
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Shift complete");
    this->set_torque_request(TorqueRequest::None, 0);
    this->shifting = false;
    vTaskDelete(nullptr);
}

void Gearbox::inc_subprofile()
{
    portENTER_CRITICAL(&this->profile_mutex);
    if (this->current_profile != nullptr)
    {
        this->current_profile->increment_subprofile();
    }
    portEXIT_CRITICAL(&this->profile_mutex);
}

void Gearbox::controller_loop()
{
    bool lock_state = false;
    ShifterPosition last_position = ShifterPosition::SignalNotAvailable;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "GEARBOX START!");
    uint64_t expire_check = esp_timer_get_time() + 100000; // 100ms
    while (esp_timer_get_time() < expire_check)
    {
        this->shifter_pos = egs_can_hal->get_shifter_position(esp_timer_get_time() / 1000, 250);
        last_position = this->shifter_pos;
        if (this->shifter_pos == ShifterPosition::P || this->shifter_pos == ShifterPosition::N)
        {
            egs_can_hal->set_safe_start(true);
            break; // Default startup, OK
        }
        else if (this->shifter_pos == ShifterPosition::D)
        { // Car is in motion forwards!
            this->actual_gear = GearboxGear::Fifth;
            this->target_gear = GearboxGear::Fifth;
            this->gear_disagree_count = 20; // Set disagree counter to non 0. This way gearbox must calculate ratio
        }
        else if (this->shifter_pos == ShifterPosition::R)
        { // Car is in motion backwards!
            this->actual_gear = GearboxGear::Reverse_Second;
            this->target_gear = GearboxGear::Reverse_Second;
        }
        vTaskDelay(5);
    }

    uint64_t last_output_measure_time = esp_timer_get_time() / 1000;
    int old_output_rpm = this->sensor_data.output_rpm;
    while (1)
    {
        uint64_t now = esp_timer_get_time() / 1000;
        this->sensor_data.current_timestamp_ms = now;
        if (this->diag_stop_control)
        {
            vTaskDelay(50);
            continue;
        }

        bool can_read_input = this->calc_input_rpm(&sensor_data.input_rpm);
        bool can_read_output = this->calc_output_rpm(&this->sensor_data.output_rpm, now);
        //ESP_LOGI("MAIN", "%d %d", can_read_input, can_read_output);
        bool can_read = can_read_input && can_read_output;
        if (can_read)
        {
            if (now - last_output_measure_time > 250)
            {
                this->sensor_data.d_output_rpm = this->sensor_data.output_rpm - old_output_rpm;
                old_output_rpm = this->sensor_data.output_rpm;
                last_output_measure_time = now;
            }
            if (this->sensor_data.output_rpm > 0 && this->sensor_data.input_rpm > 0)
            {
                // Store our ratio
                this->sensor_data.gear_ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
                this->shadow_ratio_n2 = (float)this->rpm_reading.n2_raw / (float)this->sensor_data.output_rpm;
                this->shadow_ratio_n3 = (float)this->rpm_reading.n3_raw / (float)this->sensor_data.output_rpm;
            }
            if (!shifting && this->sensor_data.output_rpm > MIN_RATIO_CALC_RPM && this->sensor_data.input_rpm > MIN_RATIO_CALC_RPM)
            {
                if (is_fwd_gear(this->actual_gear))
                {
                    if (calcGearFromRatio(false) && this->est_gear_idx != 0)
                    {
                        // Compare gears
                        GearboxGear estimate = gear_from_idx(this->est_gear_idx);
                        if (estimate != this->actual_gear)
                        {
                            gear_disagree_count++;
                            if (gear_disagree_count >= 50)
                            {
                                this->actual_gear = estimate; // DID NOT SHIFT!
                                this->target_gear = estimate;
                                this->last_fwd_gear = estimate;
                            }
                        }
                        else
                        {
                            gear_disagree_count = 0;
                        }
                    }
                }
                else
                {
                    gear_disagree_count = 0;
                }
            }
            else
            {
                gear_disagree_count = 0;
            }
        }
        else
        {
            can_read = false;
            gear_disagree_count = 0;
        }
        if (can_read && this->sensor_data.output_rpm >= 100)
        {
            bool rev = !is_fwd_gear(this->target_gear);
            if (!this->calcGearFromRatio(rev))
            {
                // ESP_LOG_LEVEL(ESP_LOG_ERROR, "GEARBOX", "GEAR RATIO IMPLAUSIBLE");
            }
        }
        uint8_t p_tmp = egs_can_hal->get_pedal_value(now, 1000);
        if (p_tmp != 0xFF)
        {
            this->sensor_data.pedal_pos = p_tmp;
        }
        sensor_data.is_braking = egs_can_hal->get_is_brake_pressed(now, 1000);
        this->sensor_data.engine_rpm = egs_can_hal->get_engine_rpm(now, 1000);
        if (this->sensor_data.engine_rpm == UINT16_MAX)
        {
            this->sensor_data.engine_rpm = 0;
        }
        // Update solenoids, only if engine RPM is OK
        if (this->sensor_data.engine_rpm > 500)
        {
            if (!shifting)
            { // If shifting then shift manager has control over MPC working
                this->mpc_working = pressure_mgr->find_working_mpc_pressure(this->actual_gear);
            }
            this->pressure_mgr->set_target_mpc_pressure(this->mpc_working);
        }
        if (Sensors::parking_lock_engaged(&lock_state) == ESP_OK)
        {
            if (lock_state) {
                this->pressure_mgr->set_target_spc_pressure(this->mpc_working+100);
                sol_y4->write_pwm_12_bit(1024);
            }
            egs_can_hal->set_safe_start(lock_state);
            this->shifter_pos = egs_can_hal->get_shifter_position(now, 1000);
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
                this->shifter_pos == ShifterPosition::ONE)
            {
                if (this->shifter_pos != last_position)
                {
                    if (lock_state)
                    {
                        if (this->shifter_pos == ShifterPosition::P)
                        {
                            this->target_gear = GearboxGear::Park;
                            last_position = this->shifter_pos;
                            if (this->pressure_mgr != nullptr)
                            {
                                this->pressure_mgr->save();
                            }
                            this->shift_reporter->save();
                            this->tcc->save();
                        }
                        else if (this->shifter_pos == ShifterPosition::N)
                        {
                            this->target_gear = GearboxGear::Neutral;
                            last_position = this->shifter_pos;
                        }
                    }
                    else
                    {
                        // Drive or R!
                        if (this->shifter_pos == ShifterPosition::R)
                        {
                            this->target_gear = this->start_second ? GearboxGear::Reverse_Second : GearboxGear::Reverse_First;
                            last_position = this->shifter_pos;
                        }
                        else if (this->shifter_pos == ShifterPosition::D || this->shifter_pos == ShifterPosition::N_D)
                        {
                            // Some shift levers can be semi broken
                            // and can switch between ND and D randomly whilst in motion
                            // handle that case here by checking current fwd gear first
                            // If current gear is also fwd, ignore!
                            if (!is_fwd_gear(this->actual_gear) && !is_fwd_gear(this->target_gear))
                            {
                                this->target_gear = this->start_second ? GearboxGear::Second : GearboxGear::First;
                            }
                            last_position = this->shifter_pos;
                        }
                    }
                }
            }
        }
        if (this->sensor_data.engine_rpm > 500)
        {
            if (can_read && is_fwd_gear(this->actual_gear))
            {
                // Check our range restict (Only for TRRS)
                switch (egs_can_hal->get_shifter_position(now, 250)) { // Don't use shifter_pos, as that only registers D. Query raw selector pos
                    case ShifterPosition::FOUR:
                        this->restrict_target = GearboxGear::Fourth;
                        break;
                    case ShifterPosition::THREE:
                        this->restrict_target = GearboxGear::Third;
                        break;
                    case ShifterPosition::TWO:
                        this->restrict_target = GearboxGear::Second;
                        break;
                    case ShifterPosition::ONE:
                        this->restrict_target = GearboxGear::First;
                        break;
                    default:
                        this->restrict_target = GearboxGear::Fifth;
                        break;
                }
                // Seek up the restriction target if the RPM is too high for the current gear!
                // Seek up to Fifth
                while (this->restrict_target != GearboxGear::Fifth && calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, this->restrict_target, &this->gearboxConfig) > this->redline_rpm)
                {
                    this->restrict_target = next_gear(this->restrict_target);
                }

                // In gear, not shifting, and no ratio mismatch
                if (!shifting && this->actual_gear == this->target_gear && gear_disagree_count == 0)
                {
                    // Enter critical ISR section
                    portENTER_CRITICAL(&this->profile_mutex);
                    AbstractProfile *p = this->current_profile;
                    // Exit critical
                    portEXIT_CRITICAL(&this->profile_mutex);
                    // Check if profile is loaded
                    if (p != nullptr)
                    {
                        // Ask the current drive profile if it thinks, given the current
                        // data, if the car should up/downshift
                        if (this->restrict_target > this->actual_gear && p->should_upshift(this->actual_gear, &this->sensor_data))
                        {
                            this->ask_upshift = true; // Upshift takes priority
                        } else if (this->restrict_target < this->actual_gear || p->should_downshift(this->actual_gear, &this->sensor_data)) {
                            this->ask_downshift = true; // Downshift is secondary
                        }
                    }
                    if (this->ask_upshift && this->actual_gear < GearboxGear::Fifth)
                    {
                        // Check RPMs
                        GearboxGear next = next_gear(this->actual_gear);
                        // Second gear shift defaults to OK as we can safely start in second (For C/W mode)
                        if (next == GearboxGear::Second || calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, next, &this->gearboxConfig) > MIN_WORKING_RPM + 100)
                        {
                            this->target_gear = next;
                        }
                    }
                    else if (this->ask_downshift && this->actual_gear > GearboxGear::First)
                    {
                        // Check RPMs
                        GearboxGear prev = prev_gear(this->actual_gear);
                        if (calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, prev, &this->gearboxConfig) < this->redline_rpm - 500)
                        {
                            this->target_gear = prev;
                        }
                    }
                }
                // Request processed. Cancel the requests. Put this outside here so that if there is a ratio mismatch, paddles are ignored
                this->ask_downshift = false;
                this->ask_upshift = false;

                if (is_fwd_gear(this->target_gear))
                    {
                        this->sensor_data.tcc_slip_rpm = sensor_data.engine_rpm - sensor_data.input_rpm;
                        if (this->tcc != nullptr)
                        {
                            this->tcc->update(this->actual_gear, this->target_gear, this->pressure_mgr, this->current_profile, &this->sensor_data, this->shifting);
                            egs_can_hal->set_clutch_status(this->tcc->get_clutch_state());
                        }
                    }
            } else { // Cannot read, or not in foward gear!
                        this->tcc_percent = 0;
                        this->pressure_mgr->set_target_tcc_pressure(0);
                        egs_can_hal->set_clutch_status(ClutchStatus::Open);
                        // sol_tcc->write_pwm_12_bit(0);
                    }
            // Not shifting, but target has changed! Spawn a shift thread!
            if (this->target_gear != this->actual_gear && !this->shifting)
            {
                xTaskCreatePinnedToCore(Gearbox::start_shift_thread, "Shift handler", 8192, this, 10, &this->shift_task, 1);
            }
        }
        else if (!shifting || asleep)
        {
            mpc_cc->set_target_current(0);
            spc_cc->set_target_current(0);
            sol_tcc->write_pwm_12_bit(0);
            sol_y3->write_pwm_12_bit(0);
            sol_y4->write_pwm_12_bit(0);
            sol_y5->write_pwm_12_bit(0);
        }
        int16_t tmp_atf = 0;
        if (lock_state || !Sensors::read_atf_temp(&tmp_atf) == ESP_OK)
        {
            // Default to engine coolant
            tmp_atf = (egs_can_hal->get_engine_coolant_temp(now, 1000));
            if (tmp_atf != INT16_MAX)
            {
                this->sensor_data.atf_temp = tmp_atf;
            }
        }
        else
        {
            if (!temp_cal)
            {
                temp_cal = true;
                temp_at_test = tmp_atf;
                if (temp_at_test != 25)
                {
                    resistance_mpc = resistance_mpc + (resistance_mpc * (((25.0 - (float)temp_at_test) * 0.393) / 100.0));
                    resistance_spc = resistance_spc + (resistance_spc * (((25.0 - (float)temp_at_test) * 0.393) / 100.0));
                }
                ESP_LOGI("GB", "Calibrated solenoids at %d C. Adjusted for 25C: SPC %.2f MPC %.2f", tmp_atf, resistance_spc, resistance_mpc);
            }
            // SPC and MPC can cause voltage swing on the ATF line, so disable
            // monitoring when shifting gears!
            if (!shifting)
            {
                this->sensor_data.atf_temp = tmp_atf;
            }
        }
        egs_can_hal->set_gearbox_temperature(this->sensor_data.atf_temp);
        egs_can_hal->set_shifter_position(this->shifter_pos);
        egs_can_hal->set_input_shaft_speed(this->sensor_data.input_rpm);
        if (this->aborting)
        {
            egs_can_hal->set_abort_shift(true);
        }
        else
        {
            egs_can_hal->set_target_gear(this->target_gear);
        }
        egs_can_hal->set_actual_gear(this->actual_gear);
        egs_can_hal->set_wheel_torque(0); // Nm

        int static_torque = egs_can_hal->get_static_engine_torque(now, 500);
        if (static_torque != INT_MAX)
        {
            this->sensor_data.static_torque = static_torque;
            if (nullptr != this->itm) {
                this->itm->update(egs_can_hal, &this->sensor_data, is_fwd_gear(this->target_gear) && is_fwd_gear(this->actual_gear));
            } else {
                this->sensor_data.input_torque = static_torque;
                egs_can_hal->set_turbine_torque_loss(0xFFFF);
            }
        }

        int driver_torque = egs_can_hal->get_driver_engine_torque(now, 500);
        if (driver_torque != INT_MAX)
        {
            this->sensor_data.driver_requested_torque = driver_torque;
        }
        int max_torque = egs_can_hal->get_maximum_engine_torque(now, 500);
        if (max_torque != INT_MAX)
        {
            this->sensor_data.max_torque = max_torque;
        }
        int min_torque = egs_can_hal->get_minimum_engine_torque(now, 500);
        if (min_torque != INT_MAX)
        {
            this->sensor_data.min_torque = min_torque;
        }
        // Wheel torque
        /*
        if (this->sensor_data.gear_ratio == 0)
        {
            // Fallback ratio for when gear ratio is actually 0
            float f;
            switch (this->target_gear)
            {
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
                f = gearboxConfig.ratios[4] * -1;
                break;
            case GearboxGear::Reverse_Second:
                f = gearboxConfig.ratios[4] * -1;
                break;
            case GearboxGear::Park:
            case GearboxGear::SignalNotAvailable:
            case GearboxGear::Neutral:
            default:
                f = 0.0;
                break;
            }
            egs_can_hal->set_wheel_torque_multi_factor(f);
        }
        else
        {
            egs_can_hal->set_wheel_torque_multi_factor(this->sensor_data.gear_ratio);
        }
        */

        // ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Torque: MIN: %3d, MAX: %3d, STAT: %3d", min_torque, max_torque, static_torque);
        //  Show debug symbols on IC
        if (this->show_upshift && this->show_downshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::RequestGearAgain);
        }
        else if (this->show_upshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::Upshift);
        }
        else if (this->show_downshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::Downshift);
        }
        else
        {
            egs_can_hal->set_display_msg(GearboxMessage::None);
        }

        // Lastly, set display gear
        portENTER_CRITICAL(&this->profile_mutex);
        if (this->current_profile != nullptr)
        {
            egs_can_hal->set_drive_profile(this->current_profile->get_profile());
            if (this->flaring && SBS_CURRENT_SETTINGS.f_shown_if_flare)
            {
                // Takes president
                egs_can_hal->set_display_msg(GearboxMessage::None);
                egs_can_hal->set_display_gear(GearboxDisplayGear::Failure, false);
            }
            else
            {
                egs_can_hal->set_display_gear(this->current_profile->get_display_gear(this->target_gear, this->actual_gear), this->current_profile == manual);
            }
        }
        portEXIT_CRITICAL(&this->profile_mutex);
        vTaskDelay(20 / portTICK_PERIOD_MS); // 50 updates/sec!
    }
}

bool Gearbox::calc_input_rpm(uint16_t *dest)
{
    bool ok = false;
    bool conduct_sanity_check = gear_disagree_count == 0 &&
                                (this->actual_gear == this->target_gear) && (                                                 // Same gear (Not shifting)
                                                                                (this->actual_gear == GearboxGear::Second) || // And in 2..
                                                                                (this->actual_gear == GearboxGear::Third) ||  // .. or 3 ..
                                                                                (this->actual_gear == GearboxGear::Fourth)    // .. or 4
                                                                            );
    if (Sensors::read_input_rpm(&rpm_reading, conduct_sanity_check) == ESP_OK)
    {
        ok = true;
        this->sensor_data.input_rpm = rpm_reading.calc_rpm;
    }
    return ok;
}

bool Gearbox::calc_output_rpm(uint16_t *dest, uint64_t now)
{
    bool result = true;
    if (VEHICLE_CONFIG.io_0_usage == 1 && VEHICLE_CONFIG.input_sensor_pulses_per_rev != 0) {
        result = Sensors::read_output_rpm(dest);
    } else {
        this->sensor_data.rl_wheel = egs_can_hal->get_rear_left_wheel(now, 500);
        this->sensor_data.rr_wheel = egs_can_hal->get_rear_right_wheel(now, 500);
        if ((WheelDirection::SignalNotAvailable != sensor_data.rl_wheel.current_dir) || (WheelDirection::SignalNotAvailable != sensor_data.rr_wheel.current_dir))
        {
            float rpm = 0.0F;
            if (WheelDirection::SignalNotAvailable == sensor_data.rl_wheel.current_dir)
            {
                // Right OK
                ESP_LOG_LEVEL(ESP_LOG_WARN, "CALC_OUTPUT_RPM", "Could not obtain left wheel RPM, trusting the right one!");
                rpm = sensor_data.rr_wheel.double_rpm;
            }
            else if (WheelDirection::SignalNotAvailable == sensor_data.rr_wheel.current_dir)
            {
                // Left OK
                ESP_LOG_LEVEL(ESP_LOG_WARN, "CALC_OUTPUT_RPM", "Could not obtain right wheel RPM, trusting the left one!");
                rpm = sensor_data.rl_wheel.double_rpm;
            }
            else
            { // Both sensors OK!
                rpm = (abs(sensor_data.rl_wheel.double_rpm) + abs(sensor_data.rr_wheel.double_rpm)) / 2;
            }
            rpm *= this->diff_ratio_f;
            rpm /= 2;
            //ESP_LOGI("ORPM", "Output RPM %.1f (%d %d)", rpm, sensor_data.rl_wheel.double_rpm, sensor_data.rr_wheel.double_rpm);
            // Car is 4Matic, but only calcualte extra if it is not a 1:1 4Matic.
            // Cars tend to be always 1:1 (Simple 4WD), but vehicles like G-Wagon / Sprinter will have a variable transfer case
            if (VEHICLE_CONFIG.is_four_matic && (VEHICLE_CONFIG.transfer_case_high_ratio != 1000 || VEHICLE_CONFIG.transfer_case_low_ratio != 1000))
            {
                switch (egs_can_hal->get_transfer_case_state(now, 500))
                {
                case TransferCaseState::Hi:
                    rpm *= ((float)(VEHICLE_CONFIG.transfer_case_high_ratio) / 1000.0);
                    result = true;
                    break;
                case TransferCaseState::Low:
                    rpm *= ((float)(VEHICLE_CONFIG.transfer_case_low_ratio) / 1000.0);
                    result = true;
                    break;
                case TransferCaseState::Neither:
                    ESP_LOG_LEVEL(ESP_LOG_WARN, "CALC_OUTPUT_RPM", "Cannot calculate output RPM. VG is in Neutral!");
                    result = false;
                    break;
                case TransferCaseState::Switching:
                    ESP_LOG_LEVEL(ESP_LOG_WARN, "CALC_OUTPUT_RPM", "Cannot calculate output RPM. VG is switching!");
                    result = false;
                    break;
                case TransferCaseState::SNA:
                    ESP_LOG_LEVEL(ESP_LOG_ERROR, "CALC_OUTPUT_RPM", "No signal from VG! Cannot read output rpm");
                    result = false;
                    break;
                }
            }
            if (result)
            {
                *dest = rpm;
            }
        }
        else
        {
            result = false;
        }
    }
    return result;
}

bool Gearbox::calcGearFromRatio(bool is_reverse)
{
    float ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
    //ESP_LOGI("CGFR", "R %.3f", ratio);
    if (is_reverse)
    {
        ratio *= -1;
        for (uint8_t i = 0; i < 2; i++)
        { // Scan the 2 reverse gears
            GearRatioInfo limits = gearboxConfig.bounds[i + 5];
            if (ratio >= limits.ratio_min_drift && ratio <= limits.ratio_max_drift)
            {
                //ESP_LOGI("CGFR", "G %d", i+1);
                this->est_gear_idx = i + 1;
                return true;
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 5; i++)
        { // Scan the 5 forwards gears
            GearRatioInfo limits = gearboxConfig.bounds[i];
            if (ratio >= limits.ratio_min_drift && ratio <= limits.ratio_max_drift)
            {
                this->est_gear_idx = i + 1;
                return true;
            }
        }
    }
    this->est_gear_idx = 0;
    return false;
}

Gearbox *gearbox = nullptr;
