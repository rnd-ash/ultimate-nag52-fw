#include "gearbox.h"
#include "common_structs_ops.h"
#include "nvs/eeprom_config.h"
#include "adv_opts.h"
#include <tcu_maths.h>
#include "solenoids/constant_current.h"
#include "speaker.h"
#include "esp_timer.h"

#define max(a, b)           \
    ({                      \
        typeof(a) _a = (a); \
        typeof(b) _b = (b); \
        _a > _b ? _a : _b;  \
    })

// ONLY FOR FORWARD GEARS!
int calc_input_rpm_from_req_gear(const int output_rpm, const GearboxGear req_gear, const GearboxConfiguration* gb_config)
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
    // IMPORTANT - Set the Ratio2/Ratio1 multiplier for the sensor RPM reading algorithm!
    Sensors::set_ratio_2_1(nag_settings->ratio_1/nag_settings->ratio_2);

    this->pressure_mgr = new PressureManager(&this->sensor_data, this->gearboxConfig.max_torque);
    this->tcc = new TorqueConverter(this->gearboxConfig.max_torque);
    this->shift_reporter = new ShiftReporter();
    this->itm = new InputTorqueModel();
    this->shift_adapter = new ShiftAdaptationSystem(&this->gearboxConfig);
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
    xTaskCreatePinnedToCore(PressureManager::start_pm_internal, "PM", 8192, static_cast<void *>(this->pressure_mgr), 10, nullptr, 1);
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

ClutchSpeeds Gearbox::diag_get_clutch_speeds() {
    return ClutchSpeedModel::get_clutch_speeds_debug(
        sensor_data.output_rpm,
        this->rpm_reading,
        this->last_motion_gear,
        this->actual_gear,
        this->target_gear,
        this->gearboxConfig.bounds
    );
}

ShiftReportSegment Gearbox::collect_report_segment(uint64_t start_time) {
    return ShiftReportSegment {
        .static_torque = sensor_data.static_torque,
        .driver_torque = sensor_data.driver_requested_torque,
        .egs_req_torque = (int16_t)((this->output_data.torque_req_type == TorqueRequest::None) ? INT16_MAX : (int16_t)(this->output_data.torque_req_amount)),
        .engine_rpm = sensor_data.engine_rpm,
        .input_rpm = sensor_data.input_rpm,
        .output_rpm = sensor_data.output_rpm,
        .mpc_pressure = this->pressure_mgr->get_targ_mpc_clutch_pressure(),
        .spc_pressure = this->pressure_mgr->get_targ_spc_clutch_pressure(),
        .timestamp = (uint16_t)(sensor_data.current_timestamp_ms-start_time)
    };
}


int Gearbox::calc_torque_limit(ProfileGearChange change, uint16_t shift_speed_ms) {
    float ped_trq = MAX(sensor_data.driver_requested_torque, sensor_data.static_torque);
    float multi_reduction = scale_number(ped_trq, &SBS_CURRENT_SETTINGS.torque_reduction_factor_input_torque);
    multi_reduction *= scale_number(shift_speed_ms, &SBS_CURRENT_SETTINGS.torque_reduction_factor_shift_speed);
    int restricted = ped_trq - (ped_trq*multi_reduction);
    if (restricted > gearboxConfig.max_torque/2) {
        restricted = gearboxConfig.max_torque/2;
    }
    return restricted;
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
        ShiftData sd = pressure_mgr->get_basic_shift_data(&this->gearboxConfig, req_lookup, chars);
        if (this->last_shift_circuit == sd.shift_circuit) { // Same shift solenoid
            while (sensor_data.current_timestamp_ms - sensor_data.last_shift_time < 500) {
                vTaskDelay(10);
            }
            // Recalculate
            sd.bleed_pressure /= 2; // Even lower pressure in bleed phase
        }
        this->last_shift_circuit = sd.shift_circuit;
        ShiftStage current_stage = ShiftStage::Bleed;
        bool process_shift = true;
        sr.start_reading = this->collect_report_segment(shift_start_time);
        float d_trq = 0;
        float clamped_trq = 0;

        // For all the stages, we only need these numbers for interpolation
        // calculate prefill data now, as we need this to set MPC offset
        PrefillData prefill_data = pressure_mgr->make_fill_data(req_lookup);

        AdaptShiftRequest adaptation_req;
        bool adapting_shift = false;

        // When a phase starts, its X value (Time) will always be 0
        float phase_targ_spc = 0;
        float phase_targ_mpc = 0;

        float phase_x_ramp_time = 0;
        float phase_total_time = 50;

        float prev_spc = 50;
        float prev_mpc = 0;

        // Current Y values
        float current_mpc = 0;
        float current_spc = 0;

        uint32_t total_elapsed = 0;
        uint32_t phase_elapsed = 0;

        int shift_progress_percentage = 0;
        int shift_progress_percentage_engine = 0;
        int trq_now = sensor_data.static_torque;
        // Test - set trq req to trq/2 now preshift (prevent engine overrunning with TCC opening)
        int t_d_rpm = 10; // per 20ms (Changed when overlap phase begins)
        float spc_delta = 0;
        int start_nm = sensor_data.static_torque;
        int16_t prefill_torque_requested = INT16_MAX;
        int delta_trq = 0;
        uint32_t prefill_adapt_flags = this->shift_adapter->check_prefill_adapt_conditions_start(&this->sensor_data, req_lookup, &prefill_torque_requested);
        if (prefill_torque_requested != INT16_MAX) {
            // Use prefill test torque so we modify the same cell in the adapt map
            prefill_data.fill_pressure_on_clutch += this->shift_adapter->get_prefill_pressure_offset(prefill_torque_requested, get_clutch_to_apply(req_lookup));
        } else {
            prefill_data.fill_pressure_on_clutch += this->shift_adapter->get_prefill_pressure_offset(sensor_data.input_torque, get_clutch_to_apply(req_lookup));// Use torque now
        }
        int prefill_pre_adapt = prefill_data.fill_pressure_on_clutch;
        if (prefill_adapt_flags != 0) {
            ESP_LOGI("SHIFT", "Prefill adapting is not allowed. Reason flag is 0x%08X", (int)prefill_adapt_flags);
        } else {
            ESP_LOGI("SHIFT", "Prefill adapting is allowed. Torque limit is %d Nm", prefill_torque_requested);
        }
        // Limit torque if adapter doesn't ask
        if (prefill_torque_requested == INT16_MAX) {
            prefill_torque_requested = MIN(250.0, sensor_data.static_torque*0.75);
        }
        bool tcc_completed = false;
        int trq_mpc = 0;
        uint64_t confirm_ticks = 0;
        // 4 shift types
        // Upshift, engine decreasing  (UED)
        // Downshift, engine decreasing (DED)
        // Upshit, engine increasing (UEI)
        // Downshift, engine increasing (DEI)
        // 
        // Each shift has a different pressure ramp type,
        // since the timings and forces on the clutches are different
        while(process_shift) {
            bool resistive_shift = false;
            if (is_upshift && sensor_data.static_torque > 0) {
                resistive_shift = true;
            } else if (!is_upshift && sensor_data.static_torque < 0) {
                resistive_shift = true;
            }

            // Shifter moved mid shift!
            if (!is_shifter_in_valid_drive_pos(this->shifter_pos)) {
                process_shift = false;
                result = false;
                break;
            }

            // Grab ratio informations
            int rpm_target_gear = calc_input_rpm_from_req_gear(sensor_data.output_rpm, this->target_gear, &this->gearboxConfig);
            int rpm_current_gear = calc_input_rpm_from_req_gear(sensor_data.output_rpm, this->actual_gear, &this->gearboxConfig);

            int wp_current_gear = pressure_manager->find_working_mpc_pressure(this->actual_gear);
            int wp_target_gear = pressure_manager->find_working_mpc_pressure(this->target_gear);

            bool coasting_shift = 0 > sensor_data.static_torque;

            // Shift reporting
            if (MIN_RATIO_CALC_RPM < sensor_data.input_rpm) {
                shift_progress_percentage =  MIN(MAX(progress_between_targets(sensor_data.input_rpm, rpm_current_gear, rpm_target_gear),0), 100);
                shift_progress_percentage_engine = MIN(MAX(progress_between_targets(sensor_data.engine_rpm, rpm_current_gear, rpm_target_gear),0), 100);
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

                if (prefill_torque_requested != INT16_MAX) {
                    prefill_torque_requested = calc_torque_limit(req_lookup, chars.target_shift_time);
                    int trq = scale_number(shift_progress_percentage, MAX(sensor_data.driver_requested_torque, sensor_data.static_torque), prefill_torque_requested, 0, 100);
                    this->set_torque_request(TorqueRequest::LessThanFast, trq);
                }
            } else {
                // If input speed is too low, use the overlap time as a way of measuring shift progress
                int t = shift_progress_percentage;
                if (ShiftStage::Overlap == current_stage) {
                    t = linear_interp(0.0, 100.0, phase_elapsed, phase_total_time);
                }
                shift_progress_percentage = MAX(t, shift_progress_percentage);
                shift_progress_percentage_engine = shift_progress_percentage;
                this->flaring = false;
                adapting_shift = false; // Disable adaptation if RPM is too low
                this->set_torque_request(TorqueRequest::None, 0); // And also torque requests
            }


            // Check next stage (Only when in bleeding, filling)
            // Overlap phase self cancels as it has to monitor shift progress
            if (phase_elapsed > phase_total_time && current_stage < ShiftStage::Overlap) {
                current_stage = next_shift_stage(current_stage);
                // New stage
                prev_spc = current_spc;
                prev_mpc = current_mpc;
                phase_elapsed = 0;
                if (current_stage == ShiftStage::Torque && sensor_data.driver_requested_torque > gearboxConfig.max_torque/2) {
                    ESP_LOGI("SHIFT", "Super load shift, skipping torque phase");
                    current_stage = ShiftStage::Overlap;
                }
                // One time phase initial code
                if (current_stage == ShiftStage::Fill) {
                    ESP_LOGI("SHIFT", "Fill start");
                    pressure_manager->set_shift_circuit(sd.shift_circuit, true);
                    phase_total_time = prefill_data.fill_time;
                    phase_x_ramp_time = 0;
                } else if (current_stage == ShiftStage::Torque) {
                    ESP_LOGI("SHIFT", "Torque start");
                    phase_x_ramp_time = 0;
                    phase_total_time = 100;
                } else if (current_stage == ShiftStage::Overlap) {
                    this->tcc->on_shift_starting(InternalTccState::Open);
                    ESP_LOGI("SHIFT", "Overlap start");
                    if (prefill_adapt_flags == 0) {
                        this->shift_adapter->on_overlap_start(sensor_data.current_timestamp_ms, chars.target_shift_time, shift_progress_percentage);
                    }
                    // SPC increases
                    // Calculate target D_RPM
                    t_d_rpm = (rpm_target_gear-rpm_current_gear)/(chars.target_shift_time/100.0);
                    ESP_LOGI("SHIFT", "RPM should change by %d/100ms for target shift speed", t_d_rpm);
                    phase_total_time = (chars.target_shift_time*2)+SBS_CURRENT_SETTINGS.shift_timeout_coasting; //(No ramping) (Worse case time)
                    phase_x_ramp_time = 0;
                    spc_delta = 0;
                    phase_targ_spc = prefill_data.fill_pressure_on_clutch;
                    prev_spc = prefill_data.fill_pressure_on_clutch;
                }
            }
            
            if (current_stage == ShiftStage::Bleed) {
                phase_targ_spc = 50;
                phase_targ_mpc = wp_current_gear;
            } else if (current_stage == ShiftStage::Fill) {
                bool was_adapting = prefill_adapt_flags == 0;
                if (prefill_adapt_flags == 0) {
                    prefill_adapt_flags = this->shift_adapter->check_prefill_adapt_conditions_shift(&this->sensor_data, egs_can_hal);
                }
                if (was_adapting && prefill_adapt_flags != 0) {
                    ESP_LOGW("SHIFT", "Adapting was cancelled. Reason flag: 0x%08X", (int)prefill_adapt_flags);
                }
                phase_targ_spc = prefill_data.fill_pressure_on_clutch;
                phase_targ_mpc = wp_current_gear + (prefill_data.fill_pressure_on_clutch/2.0);
            } else if (current_stage == ShiftStage::Torque) { // Just for conformation
                // Make MPC and SPC equal
                phase_targ_spc = prev_spc = prefill_data.fill_pressure_on_clutch/2.0;
                phase_targ_mpc = wp_current_gear + (prefill_data.fill_pressure_on_clutch/4.0);
                trq_mpc = phase_targ_mpc;
            } else if (current_stage == ShiftStage::Overlap) {
                // TODO MPC behaviour should change depending on resistance of releasing clutch
                phase_targ_mpc = prev_mpc = linear_interp(trq_mpc, 500, phase_elapsed, chars.target_shift_time*2.0);
                // To adjust on the fly in realtime, we have to modify both target and prev
                if (prefill_adapt_flags == 0) { // Adapt1ing (Not shifting yet!)
                    this->shift_adapter->do_prefill_overlap_check(sensor_data.current_timestamp_ms, flaring, shift_progress_percentage);
                }
                float step_adder = scale_number(abs(sensor_data.static_torque), 2, 30, 0, gearboxConfig.max_torque);
                step_adder *= scale_number(chars.target_shift_time, 1.0, 3.0, 1000, 100);
                step_adder *= scale_number(shift_progress_percentage, 2.0, 1.0, 0.0, 20.0);
                //if (phase_elapsed > chars.target_shift_time && shift_progress_percentage < 10) {
                //    step_adder = 15; // 1000mBar/sec
                //}
                phase_targ_spc += step_adder;
            }

            // Do lerp for pressure targets
            if (phase_x_ramp_time != 0 || phase_elapsed <= phase_x_ramp_time) { // Lerp is expensive, so only do it if we have to
                current_mpc = linear_interp(prev_mpc, phase_targ_mpc, phase_elapsed, phase_x_ramp_time);
                current_spc = linear_interp(prev_spc, phase_targ_spc, phase_elapsed, phase_x_ramp_time);
            } else {
                current_mpc = phase_targ_mpc;
                current_spc = phase_targ_spc;
            }
            pressure_mgr->set_target_spc_and_mpc_pressure(current_mpc, current_spc);

            // Timeout checking (Only in overlap)
            if (ShiftStage::Overlap == current_stage) {
                //if (shift_progress_percentage > 50 && !tcc_completed) {
                //    this->tcc->on_shift_ending();
                //    tcc_completed = true;
                //}
                if (MIN_RATIO_CALC_RPM < sensor_data.input_rpm) { // Confirmed shift!
                    if (shift_progress_percentage >= 95 || (phase_elapsed > chars.target_shift_time && sd.targ_g == this->est_gear_idx)) {
                        confirm_ticks += 1;
                    }
                    if (confirm_ticks >= 500/SHIFT_DELAY_MS) { // Wait 500ms before confirming the shift!
                        result = true;
                        process_shift = false;
                        sr.detect_shift_end_ts = (uint16_t)(sensor_data.current_timestamp_ms - shift_start_time);
                    }
                } else if (MIN_RATIO_CALC_RPM > sensor_data.input_rpm && phase_elapsed > 1000) {
                    result = true;
                    process_shift = false;
                } else if (!coasting_shift && MAX(SBS_CURRENT_SETTINGS.shift_timeout_pulling, phase_total_time*2) < phase_elapsed) { // TIMEOUT
                    result = false;
                    process_shift = false;
                } else if (coasting_shift && MAX(SBS_CURRENT_SETTINGS.shift_timeout_coasting, phase_total_time*2) < phase_elapsed) { // TIMEOUT
                    result = false;
                    process_shift = false;
                }
            }

            vTaskDelay(SHIFT_DELAY_MS / portTICK_PERIOD_MS);
            phase_elapsed += SHIFT_DELAY_MS;
            total_elapsed += SHIFT_DELAY_MS;
        }
        this->shift_adapter->debug_print_offset_array();
        // Only do max pressure phase if we shifted
        this->tcc->on_shift_starting(InternalTccState::Slipping);
        if (result) {
            ESP_LOGI("SHIFT","Max-lock start");
            // Timings
            PressureStageTiming max_p_timings = pressure_manager->get_max_pressure_timing();
            float start_spc = current_spc;
            float start_mpc = current_mpc;
            int old_spc = current_spc;
            int old_mpc = current_mpc;
            uint16_t e = 0;
            //this->tcc.
            int curr_torq_request = this->output_data.torque_req_amount;
            int eng_completion = shift_progress_percentage_engine;
            while (e < max_p_timings.hold_time + max_p_timings.ramp_time) {
                float c = linear_interp(start_spc, MAX(this->mpc_working*2.5, old_spc*2), e, max_p_timings.ramp_time);
                pressure_manager->set_target_spc_pressure(c);
                this->mpc_working = linear_interp(start_mpc, MAX(current_mpc, pressure_manager->find_working_mpc_pressure(this->target_gear)), e, max_p_timings.ramp_time);
                current_mpc = this->mpc_working;

                if (MIN_RATIO_CALC_RPM < sensor_data.input_rpm && prefill_torque_requested != UINT16_MAX) {
                    prefill_torque_requested = calc_torque_limit(req_lookup, chars.target_shift_time);
                    // RPM targets
                    int rpm_target_gear = calc_input_rpm_from_req_gear(sensor_data.output_rpm, this->target_gear, &this->gearboxConfig);
                    int rpm_current_gear = calc_input_rpm_from_req_gear(sensor_data.output_rpm, this->actual_gear, &this->gearboxConfig);
                    // Shift progress
                    shift_progress_percentage_engine = MIN(MAX(progress_between_targets(sensor_data.engine_rpm, rpm_current_gear, rpm_target_gear),0), 100);
                    int t_now = this->output_data.torque_req_amount;
                    // Torque request according to input shaft completion
                    int trq_e = MAX(t_now, scale_number(shift_progress_percentage_engine, prefill_torque_requested, MAX(0, sensor_data.driver_requested_torque), eng_completion, 100));
                    // Lerp engine and input requests based on progress of max p phase to smooth things
                    int trq = linear_interp(trq_e,  MAX(0, sensor_data.driver_requested_torque), e, max_p_timings.hold_time + max_p_timings.ramp_time);
                    this->set_torque_request(TorqueRequest::LessThanFast, trq);
                    // Dynamic calculation of torque request
                } else {
                    this->set_torque_request(TorqueRequest::None, 0);
                }
                vTaskDelay(20);
                e += 20;
            }
        }
        this->tcc->on_shift_ending();
        sr.end_reading = this->collect_report_segment(shift_start_time);
        sr.overlap_reading_size = overlap_report_size;
        sr.shift_status = result;
        sr.target_shift_speed = chars.target_shift_time;
        pressure_manager->set_spc_p_max();
        pressure_manager->set_shift_circuit(sd.shift_circuit, false);
        this->set_torque_request(TorqueRequest::None, 0);
        this->abort_shift = false;
        this->sensor_data.last_shift_time = sensor_data.current_timestamp_ms;
        this->flaring = false;
        if (result) { // Only set gear on conformation!
            ESP_LOGI("SHIFT", "SHIFT OK!");
            this->actual_gear = gear_from_idx(sd.targ_g);
        } else {
            if (!is_shifter_in_valid_drive_pos(this->shifter_pos)) {
                ESP_LOGE("SHIFT", "Shift failed due to selector moving");
                this->target_gear = GearboxGear::Neutral;
                this->actual_gear = GearboxGear::Neutral;
            } else {
                ESP_LOGE("SHIFT", "Shift failed! End ratio is %.2f", (float)sensor_data.gear_ratio/100.0);
                this->target_gear = this->actual_gear;
            }
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
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
            uint16_t elapsed = 0;
            this->mpc_working = 1000;
            while (elapsed <= 250)
            {
                pressure_mgr->set_target_spc_pressure(spc);
                elapsed += 20;
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            if (activate_y3) { sol_y3->write_pwm_12_bit(1024); }
            while (spc <= this->mpc_working) {
                pressure_mgr->set_target_spc_pressure(spc);
                spc+=10;
                vTaskDelay(10);
            }
            vTaskDelay(250);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
            if (activate_y3) { sol_y3->write_pwm_12_bit(0); }
            pressure_mgr->set_spc_p_max();
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
                            if (this->shift_adapter != nullptr)
                            {
                                this->shift_adapter->save();
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
                        if (next == GearboxGear::Second || calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, next, &this->gearboxConfig) > TCC_CURRENT_SETTINGS.min_locking_rpm + 100)
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
                        egs_can_hal->set_clutch_status(TccClutchStatus::Open);
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
        //if (sensor_data.output_rpm == 0 && this->current_profile == manual) {
        //    // Want to launch
        //    if (egs_can_hal->get_is_brake_pressed(now, 500) && sensor_data.pedal_pos > 0) {
        //        if (sensor_data.engine_rpm > 2000) {
        //            egs_can_hal->set_torque_request(TorqueRequest::LessThan, 10.0);
        //        } else {
        //            egs_can_hal->set_torque_request(TorqueRequest::None, 0.0);
        //        }
        //    } else {
        //        egs_can_hal->set_torque_request(TorqueRequest::None, 0.0);
        //    }
        //}
        
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
