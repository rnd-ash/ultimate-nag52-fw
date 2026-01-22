#include "gearbox.h"
#include "common_structs_ops.h"
#include "nvs/eeprom_config.h"
#include "adv_opts.h"
#include <tcu_maths.h>
#include "speaker.h"
#include "clock.hpp"
#include "nvs/device_mode.h"
#include "egs_calibration/calibration_structs.h"
#include "firstorder_average.h"
#include "shifting_algo/s_algo.h"
#include "shifting_algo/shift_crossover.h"
#include "shifting_algo/shift_release.h"
#include "tcu_io/tcu_io.hpp"

#define SBS SBS_CURRENT_SETTINGS

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

Gearbox::Gearbox(Shifter *shifter) : shifter(shifter), kickdown(), brake_pedal()
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
        .input_torque = 0,
        .converted_torque = 0,
        .converted_driver_torque = 0,
        .indicated_torque = 0,
        .max_torque = 0,
        .min_torque = 0,
        .last_shift_time = 0,
        .gear_ratio = 0.0F,
        .kickdown_pressed = false,
        .brake_pressed = false,
    };
    this->output_data = OutputData {
        .torque_req_amount = 0,
        .ctrl_type = TorqueRequestControlType::None,
        .bounds = TorqueRequestBounds::LessThan,
    };

    float r1 = ((float)(MECH_PTR->ratio_table[1]))/1000.0;
    float r2 = ((float)(MECH_PTR->ratio_table[2]))/1000.0;
    float r3 = ((float)(MECH_PTR->ratio_table[3]))/1000.0;
    float r4 = ((float)(MECH_PTR->ratio_table[4]))/1000.0;
    float r5 = ((float)(MECH_PTR->ratio_table[5]))/1000.0;
    float rr1 = ((float)(MECH_PTR->ratio_table[6])*-1)/1000.0;
    float rr2 = ((float)(MECH_PTR->ratio_table[7])*-1)/1000.0;

    this->gearboxConfig.max_torque = 330;
    if (MECH_PTR->gb_ty == 0) {
        this->gearboxConfig.max_torque = 580;
    }
    this->gearboxConfig.bounds[0] = GearRatioInfo { // 1st 
        .ratio_max_drift = r1*(float)1.1,
        .ratio = r1,
        .ratio_min_drift = r1*(float)0.9,
    };
    this->gearboxConfig.bounds[1] = GearRatioInfo { // 2nd 
        .ratio_max_drift = r2*(float)1.1,
        .ratio = r2,
        .ratio_min_drift = r2*(float)0.9,
    };
    this->gearboxConfig.bounds[2] = GearRatioInfo { // 3rd 
        .ratio_max_drift = r3*(float)1.1,
        .ratio = r3,
        .ratio_min_drift = r3*(float)0.9,
    };
    this->gearboxConfig.bounds[3] = GearRatioInfo { // 4th 
        .ratio_max_drift = r4*(float)1.1,
        .ratio = r4,
        .ratio_min_drift = r4*(float)0.9,
    };
    this->gearboxConfig.bounds[4] = GearRatioInfo { // 5th 
        .ratio_max_drift = r5*(float)1.1,
        .ratio = r5,
        .ratio_min_drift = r5*(float)0.9,
    };
    this->gearboxConfig.bounds[5] = GearRatioInfo { // R1 
        .ratio_max_drift = rr1*(float)1.1,
        .ratio = rr1,
        .ratio_min_drift = rr1*(float)0.9,
    };
    this->gearboxConfig.bounds[6] = GearRatioInfo { // R2 
        .ratio_max_drift = rr2*(float)1.1,
        .ratio = rr2,
        .ratio_min_drift = rr2*(float)0.9,
    };
    // IMPORTANT - Set the Ratio2/Ratio1 multiplier for the sensor RPM reading algorithm!
    TCUIO::set_2_1_ratio(r1/r2);

    this->pressure_mgr = new PressureManager(&this->sensor_data, this->gearboxConfig.max_torque);
    this->tcc = new TorqueConverter(this->gearboxConfig.max_torque);
    this->shift_adapter = new ShiftAdaptationSystem(&this->gearboxConfig);
    pressure_manager = this->pressure_mgr;
    // Wait for solenoid routine to complete
    if (!Solenoids::init_routine_completed())
    {
        vTaskDelay(1);
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
    this->diff_ratio_f = (float)VEHICLE_CONFIG.diff_ratio / 1000.0;
    this->pedal_average = new FirstOrderAverage(25);
    sensor_data.pedal_delta = new FirstOrderAverage(25);


    sensor_data.pedal_smoothed = (const FirstOrderAverage*)this->pedal_average;
}

bool Gearbox::is_stationary() {
    return this->sensor_data.input_rpm < 100 && this->sensor_data.output_rpm < 100;
}

void Gearbox::set_profile(AbstractProfile *prof)
{
    if ((nullptr != prof) && ((nullptr == current_profile) || (prof != current_profile)))
    {
        // Only change if not nullptr!
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

float ratio_absolute(GearboxGear g, GearboxConfiguration* cfg) {
    switch (g) {
        case GearboxGear::First:
            return cfg->bounds[0].ratio;
        case GearboxGear::Second:
            return cfg->bounds[1].ratio;
        case GearboxGear::Third:
            return cfg->bounds[2].ratio;
        case GearboxGear::Fourth:
            return cfg->bounds[3].ratio;
        case GearboxGear::Fifth:
            return cfg->bounds[4].ratio;
        case GearboxGear::Reverse_First:
            return abs(cfg->bounds[5].ratio);
        case GearboxGear::Reverse_Second:
            return abs(cfg->bounds[6].ratio);
        default:
            return 0;
    }
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
    this->manual_shift = true;
}

void Gearbox::dec_gear_request()
{
    this->ask_upshift = false;
    this->ask_downshift = true;
    this->manual_shift = true;
}

void Gearbox::set_torque_request(TorqueRequestControlType ctrl_type, TorqueRequestBounds bounds, float amount) {
    this->output_data.torque_req_amount = amount;
    this->output_data.ctrl_type = ctrl_type;
    this->output_data.bounds = bounds;
    egs_can_hal->set_torque_request(ctrl_type, bounds, amount);
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

#define SHIFT_DELAY_MS 20     // 20ms steps
#define NUM_SCD_ENTRIES 100 / SHIFT_DELAY_MS // 100ms moving average window

ClutchSpeeds Gearbox::diag_get_clutch_speeds()
{

	return ClutchSpeedModel::get_clutch_speeds_debug(
        this->speed_sensors,
        this->last_motion_gear,
        this->actual_gear,
        this->target_gear,
        this->gearboxConfig.bounds
    );
}

ShiftReportSegment Gearbox::collect_report_segment(uint64_t start_time) {
    return ShiftReportSegment {
        .static_torque = sensor_data.converted_torque,
        .driver_torque = sensor_data.converted_driver_torque,
        .egs_req_torque = (int16_t)((this->output_data.ctrl_type == TorqueRequestControlType::None) ? INT16_MAX : (int16_t)(this->output_data.torque_req_amount)),
        .engine_rpm = sensor_data.engine_rpm,
        .input_rpm = sensor_data.input_rpm,
        .output_rpm = sensor_data.output_rpm,
        .mpc_pressure = 0, //this->pressure_mgr->get_targ_mpc_clutch_pressure(),
        .spc_pressure = 0, //this->pressure_mgr->get_targ_spc_clutch_pressure(),
        .timestamp = (uint16_t)(GET_CLOCK_TIME()-start_time)
    };
}

/**
 * @brief Used to shift between forward gears
 *
 * @return uint16_t - The actual time taken to shift gears. This is fed back into the adaptation network so it can better meet 'target_shift_duration_ms'
 */

bool Gearbox::elapse_shift(GearChange req_lookup, AbstractProfile *profile, bool manually_requested)
{
    bool result = false;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ELAPSE_SHIFT", "Shift started!");
    // This is important for all EGS compatibility
    uint8_t egs_map_idx_lookup = fwd_gearchange_egs_map_lookup_idx(req_lookup);

    if (nullptr != profile && req_lookup != GearChange::_IDLE && 0xFF != egs_map_idx_lookup)
    {
        ShiftCharacteristics chars = profile->get_shift_characteristics(req_lookup, &this->sensor_data);
        chars.target_shift_time = MAX(100, chars.target_shift_time);
        CircuitInfo sd = pressure_mgr->get_basic_shift_data(&this->gearboxConfig, req_lookup, chars);
        sd.map_idx = egs_map_idx_lookup;
        if (this->last_shift_circuit == sd.shift_circuit) { // Same shift solenoid
            while (GET_CLOCK_TIME() - sensor_data.last_shift_time < 500) {
                vTaskDelay(10);
            }
        }
        this->last_shift_circuit = sd.shift_circuit;
        bool process_shift = true;

        ShiftPressures p_now = {};
        memset(&p_now, 0, sizeof(ShiftPressures));

        uint32_t total_elapsed = 0;
        uint32_t phase_elapsed = 0;

        uint32_t prefill_adapt_flags = this->shift_adapter->check_prefill_adapt_conditions_start(&this->sensor_data, req_lookup);
        pressure_manager->register_shift_pressure_data(&p_now);

        if (prefill_adapt_flags != 0) {
            ESP_LOGI("SHIFT", "Prefill adapting is not allowed. Reason flag is 0x%08X", (int)prefill_adapt_flags);
        } else {
            ESP_LOGI("SHIFT", "Prefill adapting is allowed.");
        }

        ShiftClutchData now_cs = ClutchSpeedModel::get_shifting_clutch_speeds(this->speed_sensors, req_lookup, this->gearboxConfig.bounds);
        Clutch applying = get_clutch_to_apply(req_lookup);
        Clutch releasing = get_clutch_to_release(req_lookup);
        PrefillData prefill_data = pressure_mgr->make_fill_data(applying);

        int MOD_MAX = this->pressure_mgr->get_max_solenoid_pressure();
        int SPC_MAX = pressure_manager->get_max_shift_pressure(egs_map_idx_lookup);

        TorqueRequstData trd = {
            .ty = TorqueRequestControlType::None,
            .bounds = TorqueRequestBounds::LessThan,
            .amount = 0
        };

        ShiftInterfaceData sid = {
            .profile = profile,
            .MOD_MAX = MOD_MAX,
            .SPC_MAX = SPC_MAX,
            .shift_flags = 0,
            .change = req_lookup,
            .applying = applying,
            .releasing = releasing,
            .curr_g = this->actual_gear,
            .targ_g = this->target_gear,
            .inf = sd,
            .release_spring_on_clutch = pressure_manager->get_spring_pressure(applying),
            .release_spring_off_clutch = pressure_manager->get_spring_pressure(releasing),
            .prefill_info = prefill_data,
            .chars = chars,
            .ptr_r_clutch_speeds = &now_cs,
            .ptr_w_pressures = &p_now,
            .ptr_w_trq_req = &trd,
            .tcc = this->tcc
        };

        float inertia = ShiftHelpers::get_shift_intertia(sid.inf.map_idx);
        ShiftingAlgorithm* algo;
        if (is_upshift) {
            float threshold_m = 2.0;
            if (GearChange::_1_2 == req_lookup) {
                threshold_m = SBS.crossover_trq_thres_1_2;
            } else if (GearChange::_2_3 == req_lookup) {
                threshold_m = SBS.crossover_trq_thres_2_3;
            } else if (GearChange::_3_4 == req_lookup) {
                threshold_m = SBS.crossover_trq_thres_3_4;
            } else if (GearChange::_4_5 == req_lookup) {
                threshold_m = SBS.crossover_trq_thres_4_5;
            }
            if (sensor_data.input_torque > (threshold_m*inertia)) {
                algo = new CrossoverShift(&sid);
            } else {
                algo = new ReleasingShift(&sid);
            }
        } else {
            if (sensor_data.input_torque > inertia) {
                algo = new ReleasingShift(&sid);
            } else {
                algo = new CrossoverShift(&sid);
            }
        }

        uint8_t algo_phase_id = 0;
        GearChange pm_change = GearChange::_IDLE;
        GearboxGear pm_gear = sid.curr_g;
        bool pm_activated = false;
        while(process_shift) {
            uint32_t start_time = GET_CLOCK_TIME();
            bool stationary_shift = this->is_stationary();
            // Shifter moved mid shift!
            if (!is_shifter_in_valid_drive_pos(this->shifter_pos)) {
                process_shift = false;
                result = false;
                break;
            }

            int abs_input_torque = abs(sensor_data.input_torque);
            now_cs = ClutchSpeedModel::get_shifting_clutch_speeds(this->speed_sensors, req_lookup, this->gearboxConfig.bounds);

            // Shift reporting
            if (!stationary_shift) {
                if (now_cs.off_clutch_speed < -50 || now_cs.on_clutch_speed < -50) {
                    flaring = true;
                } else {
                    flaring = false;
                }

                this->set_torque_request(trd.ty, trd.bounds, trd.amount);
            } else {
                // If input speed is too low, use the overlap time as a way of measuring shift progress
                this->flaring = false;
                this->set_torque_request(TorqueRequestControlType::None, TorqueRequestBounds::LessThan, 0); // And also torque requests
            }

            // Algorithm has control
            uint8_t step_result = algo->step(
                algo_phase_id, 
                abs_input_torque, 
                stationary_shift,
                is_upshift,
                phase_elapsed,
                total_elapsed,
                this->pressure_mgr,
                &this->sensor_data
            );
            this->algo_feedback = algo->get_diag_feedback(algo_phase_id);

            // Update pressures
            pressure_mgr->set_target_modulating_pressure(p_now.mod_sol_req);
            pressure_mgr->set_target_shift_pressure(p_now.shift_sol_req);
            // Only if shift valve is open
            if (this->pressure_mgr->get_active_shift_circuits() != 0) {
                pm_change = sid.change;
                pm_gear = sid.targ_g;
                pm_activated = true;
            } else if (pm_activated) {
                // Shift valve closed, but circuit WAS active
                pm_gear = sid.targ_g;
                pm_change = GearChange::_IDLE;
            }
            pressure_mgr->update_pressures(pm_gear, pm_change);

            if (step_result == 0) {
                // Continue
                phase_elapsed += SHIFT_DELAY_MS;
            } else if (step_result == STEP_RES_END_SHIFT) {
                // Shift completed OK!
                result = true;
                break;
            } else if (step_result == STEP_RES_FAILURE) {
                // Shift failure!
                break; 
            } else {
                // Phase has completed, update our data
                phase_elapsed = 0;
                if (step_result == STEP_RES_NEXT) {
                    algo_phase_id += 1;
                } else {
                    algo_phase_id = step_result;
                }
            }
            uint32_t elapsed = GET_CLOCK_TIME() - start_time;
            if (elapsed < SHIFT_DELAY_MS) {
                vTaskDelay((SHIFT_DELAY_MS - elapsed) / portTICK_PERIOD_MS);
            }
            total_elapsed += SHIFT_DELAY_MS;
        }
        //this->shift_adapter->debug_print_prefill_data();
        //this->tcc->on_shift_ending();
        if (result) { // Only set gear on conformation!
            this->actual_gear = gear_from_idx(sd.targ_g);
        } else {
            if (!is_shifter_in_valid_drive_pos(this->shifter_pos)) {
                ESP_LOGE("SHIFT", "Shift failed due to selector moving");
                this->target_gear = GearboxGear::Neutral;
                this->actual_gear = GearboxGear::Neutral;
            } else {
                ESP_LOGE("SHIFT", "Shift failed! End ratio is %.2f", (float)sensor_data.gear_ratio);
                this->target_gear = this->actual_gear;
            }
        }
        pressure_manager->set_spc_p_max();
        pressure_manager->set_shift_circuit(sd.shift_circuit, false);
        pressure_manager->notify_shift_end();
        this->set_torque_request(TorqueRequestControlType::None, TorqueRequestBounds::LessThan, 0);
        this->abort_shift = false;
        this->sensor_data.last_shift_time = GET_CLOCK_TIME();
        this->flaring = false;
        memset(&this->algo_feedback, 0x00, sizeof(ShiftAlgoFeedback));
        // Reset the pedal delta
        sensor_data.pedal_delta->reset(0);        delete algo;
    } else if (GearChange::_IDLE == req_lookup) {
        ESP_LOGE("ELAPSE_SHIFT", "BUG! GearChange is IDLE");
    } else if (0xFF == egs_map_idx_lookup) {
        ESP_LOGE("ELAPSE_SHIFT", "BUG! GearChange is INVALID for FwdShift");
    } else if (nullptr == profile) {
        ESP_LOGE("ELAPSE_SHIFT", "BUG! Profile is null");
    }
    this->shift_idx = GearChange::_IDLE;
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
        float prefill = pressure_manager->p_clutch_with_coef(GearboxGear::Reverse_Second, Clutch::B3, abs(sensor_data.input_torque), CoefficientTy::Sliding);
        this->pressure_mgr->set_target_shift_pressure(prefill/2);
        this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
        //sol_y4->write_pwm_12_bit(800); // 3-4 is pulsed at 20%
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "No need to shift");
        this->actual_gear = curr_target; // Set on startup
        goto cleanup;
    }
    else if (is_controllable_gear(curr_actual) != is_controllable_gear(curr_target))
    { // This would be a garage shift, either in or out
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Garage shift");
        if (is_controllable_gear(curr_target))
        {
            bool into_reverse = GearboxGear::Reverse_First == curr_target || GearboxGear::Reverse_Second== curr_target;
            pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
            vTaskDelay(50);
            pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
            float working;
            float prefill;
            float spring;
            if (into_reverse) {
                working = pressure_manager->find_working_mpc_pressure(GearboxGear::Reverse_Second);
                prefill = pressure_manager->get_b3_prefill_pressure();
                spring = pressure_manager->get_spring_pressure(Clutch::B3);
            } else {
                working = pressure_manager->find_working_mpc_pressure(GearboxGear::Second);
                prefill = pressure_manager->make_fill_data(Clutch::B2).fill_pressure_on_clutch;
                spring = pressure_manager->get_spring_pressure(Clutch::B2);
            }
            pressure_mgr->set_target_shift_pressure(spring + (prefill));
            pressure_mgr->set_target_modulating_pressure(working);
            this->pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);
            // N/P -> R/D
            // Defaults (Start in 2nd)
            egs_can_hal->set_garage_shift_state(true, !into_reverse);
            uint16_t cycle_count = 0;
            bool completed_ok = false;

            uint16_t ramp = 0;
            uint16_t spc_step;
            uint16_t prefill_cycles;
            float spc_mul;
            if (into_reverse) {
                prefill_cycles = interpolate_float(sensor_data.atf_temp, &GAR_CURRENT_SETTINGS.prefill_time_b3, InterpType::Linear);
                spc_step = interpolate_float(sensor_data.atf_temp, &GAR_CURRENT_SETTINGS.p_ramp_b3, InterpType::Linear);
                spc_mul = GAR_CURRENT_SETTINGS.mod_mul_b3;
            } else {
                prefill_cycles = interpolate_float(sensor_data.atf_temp, &GAR_CURRENT_SETTINGS.prefill_time_b2, InterpType::Linear);
                spc_step = interpolate_float(sensor_data.atf_temp, &GAR_CURRENT_SETTINGS.p_ramp_b2, InterpType::Linear);
                spc_mul = GAR_CURRENT_SETTINGS.mod_mul_b2;
            }
            
            while(true) {
                if (this->shifter_pos == ShifterPosition::P || this->shifter_pos == ShifterPosition::N) {
                    completed_ok = false;
                    break;
                }
                if (into_reverse) {
                    working = pressure_manager->find_working_mpc_pressure(GearboxGear::Reverse_Second);
                    if (cycle_count > prefill_cycles) {
                        prefill = pressure_manager->p_clutch_with_coef(GearboxGear::Reverse_Second, Clutch::B3, abs(sensor_data.input_torque), CoefficientTy::Sliding);
                        ramp += spc_step;
                    }
                } else {
                    working = pressure_manager->find_working_mpc_pressure(GearboxGear::Second);
                    if (cycle_count > prefill_cycles) {
                        prefill = pressure_manager->p_clutch_with_coef(GearboxGear::Second, Clutch::B2, abs(sensor_data.input_torque), CoefficientTy::Sliding);
                        ramp += spc_step;
                    }
                }
                int spc = prefill + spring + ramp;
                pressure_mgr->set_target_modulating_pressure(working + (spc_mul*spc));
                pressure_mgr->set_target_shift_pressure(spc);
                this->pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);

                int turbine = this->speed_sensors.turbine;
                if (cycle_count > 50 && turbine <= 100+calc_input_rpm_from_req_gear(sensor_data.output_rpm, curr_target, &this->gearboxConfig)) {
                    completed_ok = true;
                    break;
                }
                if (cycle_count > GAR_CURRENT_SETTINGS.timeout_cycles && sensor_data.engine_rpm - sensor_data.input_rpm < 200) {
                    completed_ok = false;
                    break;
                }
                vTaskDelay(20);
                cycle_count += 1;
            }
            if (!completed_ok) {
                ESP_LOGW("SHIFT", "Garage shift aborted");
                curr_target = this->shifter_pos == ShifterPosition::P ? GearboxGear::Park : GearboxGear::Neutral;
                curr_actual = this->shifter_pos == ShifterPosition::P ? GearboxGear::Park : GearboxGear::Neutral;
                pressure_mgr->set_target_shift_pressure(prefill/2);
                //this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
            } else {
                // Shut down the 3-4 SS
                ESP_LOGI("SHIFT", "Garage shift completed OK after %d ms", cycle_count*20);
                pressure_mgr->set_spc_p_max();
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
            }
            egs_can_hal->set_garage_shift_state(false, !into_reverse);
        }
        else
        {
            // Garage shifting to N or P, we can just set the pressure back to idle
            float prefill = pressure_manager->p_clutch_with_coef(GearboxGear::Reverse_Second, Clutch::B3, abs(sensor_data.input_torque), CoefficientTy::Sliding);
            pressure_mgr->set_target_shift_pressure(prefill/2);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
        }
        if (is_fwd_gear(curr_target))
        {
            // Last forward known gear
            // This way we better handle shifting from N->D at speed!
            this->actual_gear = GearboxGear::Second;
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
                GearChange pgc;
                if (curr_target == GearboxGear::Second)
                { // 1-2
                    pgc = GearChange::_1_2;
                }
                else if (curr_target == GearboxGear::Third)
                { // 2-3
                    pgc = GearChange::_2_3;
                }
                else if (curr_target == GearboxGear::Fourth)
                { // 3-4
                    pgc = GearChange::_3_4;
                }
                else if (curr_target == GearboxGear::Fifth)
                { // 4-5
                    pgc = GearChange::_4_5;
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
                this->is_upshift = true;
                this->fwd_gear_shift = true;
                elapse_shift(pgc, prof, this->shift_req_was_manual);
                this->start_second = true;
                goto cleanup;
            }
            else
            { // Downshifting
                ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Downshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                // this->show_downshift = true;
                GearChange pgc;
                if (curr_target == GearboxGear::First)
                { // 2-1
                    pgc = GearChange::_2_1;
                    this->start_second = false;
                }
                else if (curr_target == GearboxGear::Second)
                { // 3-2
                    pgc = GearChange::_3_2;
                    this->start_second = true;
                }
                else if (curr_target == GearboxGear::Third)
                { // 4-3
                    pgc = GearChange::_4_3;
                }
                else if (curr_target == GearboxGear::Fourth)
                { // 5-4
                    pgc = GearChange::_5_4;
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
                this->is_upshift = false;
                this->fwd_gear_shift = true;
                elapse_shift(pgc, prof, this->shift_req_was_manual);
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
    this->set_torque_request(TorqueRequestControlType::None, TorqueRequestBounds::LessThan, 0);
    this->shifting = false;
    this->fwd_gear_shift = false;
    this->is_upshift = false;
    vTaskDelete(nullptr);
}

void Gearbox::controller_loop()
{
    ShifterPosition last_position = ShifterPosition::SignalNotAvailable;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "GEARBOX START!");
    uint32_t expire_check = GET_CLOCK_TIME() + 100; // 100ms
    egs_can_hal->set_safe_start(true);
    while (GET_CLOCK_TIME() < expire_check)
    {
        // Step 1. Aquire ALL Sensors
        TCUIO::update_io_layer();

        this->shifter_pos = egs_can_hal->get_shifter_position(250);
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
            egs_can_hal->set_safe_start(false);
            break;
        }
        else if (this->shifter_pos == ShifterPosition::R)
        { // Car is in motion backwards!
            this->actual_gear = GearboxGear::Reverse_Second;
            this->target_gear = GearboxGear::Reverse_Second;
            egs_can_hal->set_safe_start(false);
            break;
        } else {
            egs_can_hal->set_safe_start(true); // Unknown position, keep polling until we don't know
        }
        vTaskDelay(5);
    }
    while (1)
    {
        TCUIO::update_io_layer();
        if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {
            SOLENOID_CONTROL_EGS_SLAVE slave_rq = egs_can_hal->get_tester_req();
            sol_mpc->set_current_target(__builtin_bswap16(slave_rq.MPC_REQ));
            sol_spc->set_current_target(__builtin_bswap16(slave_rq.SPC_REQ));
            sol_tcc->set_duty(slave_rq.TCC_REQ*16); // x16 to go from 8 bit (0-255) to 12bit (0-4096)

            SENSOR_REPORT_EGS_SLAVE sensor_rpt;

            this->process_speed_sensors();


            uint8_t pll = TCUIO::parking_lock();
            int16_t tft = TCUIO::atf_temperature();
            uint16_t vbatt = TCUIO::battery_mv();

            sensor_rpt.N2_RAW = __builtin_bswap16(this->speed_sensors.n2);
            sensor_rpt.N3_RAW = __builtin_bswap16(this->speed_sensors.n3);
            sensor_rpt.TFT = tft + 40;
            sensor_rpt.VBATT = (vbatt/1000) & 0xFF;

            SOLENOID_REPORT_EGS_SLAVE sol_rpt;
            sol_rpt.MPC_CURR = __builtin_bswap16(sol_mpc->get_current());
            sol_rpt.SPC_CURR = __builtin_bswap16(sol_spc->get_current());
            sol_rpt.TCC_PWM = (sol_tcc->get_pwm_raw()/16) & 0xFF;

            UN52_REPORT_EGS_SLAVE un52_rpt;
            un52_rpt.Y3_CURR = __builtin_bswap16(sol_y3->get_current());
            un52_rpt.Y4_CURR = __builtin_bswap16(sol_y4->get_current());
            un52_rpt.Y5_CURR = __builtin_bswap16(sol_y5->get_current());
            un52_rpt.TCC_CURR = __builtin_bswap16(sol_tcc->get_current());

            egs_can_hal->set_slave_mode_reports(sol_rpt, sensor_rpt, un52_rpt);
            vTaskDelay(20);
            continue;
        }
        if (this->diag_stop_control)
        {
            vTaskDelay(50);
            continue;
        }

        // Set sensors Motor temperature (Always ran)
        int16_t coolant_temp = egs_can_hal->get_engine_coolant_temp(50);

        bool speeds_valid = this->process_speed_sensors();
        if (speeds_valid)
        {
            sensor_data.input_rpm = speed_sensors.turbine;
            sensor_data.output_rpm = speed_sensors.output;
            bool stationary = this->is_stationary();
            if (!stationary)
            {
                // Store our ratio
                this->sensor_data.gear_ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
                this->sensor_data.targ_gear_ratio = ratio_absolute(this->actual_gear, &this->gearboxConfig);

            } else {
                // Stationary so no ratios
                this->sensor_data.gear_ratio = 0.0;
                this->sensor_data.targ_gear_ratio = 0.0;
            }
            if (!shifting && !stationary && sensor_data.output_rpm > 250)
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
            speeds_valid = false;
            gear_disagree_count = 0;
        }
        if (speeds_valid && !this->is_stationary())
        {
            bool rev = !is_fwd_gear(this->target_gear);
            if (!this->calcGearFromRatio(rev))
            {
                // ESP_LOG_LEVEL(ESP_LOG_ERROR, "GEARBOX", "GEAR RATIO IMPLAUSIBLE");
            }
        }
        uint8_t p_tmp = egs_can_hal->get_pedal_value(1000);
        this->pedal_last = this->sensor_data.pedal_pos;
        if (p_tmp != 0xFF)
        {
            this->sensor_data.pedal_pos = p_tmp;
        } else {
            p_tmp = 250/4; // 25% as a fallback
        }
        this->pedal_average->add_sample(p_tmp);
        int16_t percent_delta_sec = ((int16_t)p_tmp - (int16_t)(this->pedal_last))*20.0; // 20.0 = 50 (Cycles/sec) / 2.5 (Pedal raw -> %)
        sensor_data.pedal_delta->add_sample(percent_delta_sec);
        sensor_data.brake_pressed = brake_pedal.is_brake_pedal_pressed(egs_can_hal, 250);
        sensor_data.kickdown_pressed = kickdown.is_kickdown_newly_pressed(egs_can_hal, 250);
        int tmp_rpm = 0;
        tmp_rpm = egs_can_hal->get_engine_rpm(1000);
        if (tmp_rpm == UINT16_MAX)
        {
            tmp_rpm = this->sensor_data.engine_rpm; // Sub last value!
        }
        this->sensor_data.engine_rpm = tmp_rpm;
        // Update solenoids, only if engine RPM is OK
        if (tmp_rpm > 400)
        {
            if (!shifting)
            {
                this->mpc_working = pressure_mgr->find_working_mpc_pressure(this->actual_gear);
                this->pressure_mgr->set_target_modulating_pressure(this->mpc_working);
            }
        }
        uint8_t pll = TCUIO::parking_lock();
        if (UINT8_MAX != pll)
        {
            bool lock_state = pll != 0;
            if (lock_state) {
                float prefill = pressure_manager->p_clutch_with_coef(GearboxGear::Reverse_Second, Clutch::B3, abs(sensor_data.input_torque), CoefficientTy::Sliding);
                this->pressure_mgr->set_target_modulating_pressure(this->mpc_working);
                this->pressure_mgr->set_target_modulating_pressure(prefill/2);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
                //sol_y4->write_pwm_12_bit(1024);
            }
            egs_can_hal->set_safe_start(lock_state);
            this->shifter_pos = egs_can_hal->get_shifter_position(1000);
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
        if (this->sensor_data.engine_rpm > 100)
        {
            if (speeds_valid && is_fwd_gear(this->actual_gear))
            {
                // Check our range restict (Only for TRRS)
                switch (egs_can_hal->get_shifter_position(250)) { // Don't use shifter_pos, as that only registers D. Query raw selector pos
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
                            this->manual_shift = false;
                        } else if (this->restrict_target < this->actual_gear || p->should_downshift(this->actual_gear, &this->sensor_data)) {
                            this->ask_downshift = true; // Downshift is secondary
                            this->manual_shift = false;
                        }
                    }
                    if (this->ask_upshift && this->actual_gear < GearboxGear::Fifth)
                    {
                        // Check RPMs
                        GearboxGear next = next_gear(this->actual_gear);
                        // Second gear shift defaults to OK as we can safely start in second (For C/W mode)
                        if (next == GearboxGear::Second || calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, next, &this->gearboxConfig) > 900)
                        {
                            this->target_gear = next;
                        }
                    }
                    else if ((this->ask_downshift || sensor_data.kickdown_pressed) && this->actual_gear > GearboxGear::First)
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
                this->shift_req_was_manual = this->manual_shift;
                this->manual_shift = false;

                if (is_fwd_gear(this->target_gear))
                    {
                        if (this->tcc != nullptr)
                        {
                            this->tcc->update(this->actual_gear, this->target_gear, this->pressure_mgr, this->current_profile, &this->sensor_data);
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
        else if (!shifting)
        {
            sol_mpc->set_current_target(0);
            sol_spc->set_current_target(0);
            sol_tcc->set_duty(0);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, false);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
        }
        
        int16_t tmp_atf = TCUIO::atf_temperature();
        if (INT16_MAX != tmp_atf)
        {
            this->sensor_data.atf_temp = tmp_atf;
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

        CanTorqueData trqs = egs_can_hal->get_torque_data(100);
        // CALC TORQUES
        if (INT16_MAX != trqs.m_min) { sensor_data.min_torque = trqs.m_min; }
        if (INT16_MAX != trqs.m_max) { sensor_data.max_torque = trqs.m_max; }
        if (INT16_MAX != trqs.m_ind) { sensor_data.indicated_torque = trqs.m_ind; }
        if (INT16_MAX != trqs.m_converted_static) { sensor_data.converted_torque = trqs.m_converted_static; }
        if (INT16_MAX != trqs.m_converted_driver) {
            int input_trq = InputTorqueModel::get_input_torque(
                sensor_data.engine_rpm, 
                sensor_data.input_rpm,
                trqs.m_converted_driver
            );
            sensor_data.input_torque = input_trq;
            sensor_data.converted_driver_torque = trqs.m_converted_driver;
        }
        // Override input torque if garage shifting!
        if (this->shifting && is_controllable_gear(this->target_gear) && !is_controllable_gear(this->actual_gear)) {
            // Shifting to either R or D
            int16_t pump = InputTorqueModel::get_pump_torque(sensor_data.engine_rpm, sensor_data.input_rpm);
            if (INT16_MAX != pump) {
                sensor_data.input_torque = pump;
            }
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
        float ratio_from_c_gear = ratio_absolute(this->actual_gear, &this->gearboxConfig);
        float ratio_from_t_gear = ratio_absolute(this->target_gear, &this->gearboxConfig);
        float tcc_multipler = InputTorqueModel::get_input_torque_factor(sensor_data.engine_rpm, sensor_data.input_rpm);
        this->sensor_data.tcc_trq_multiplier = tcc_multipler;
        float torque_ratio = 0; // Implausible
        if (
            ratio_from_c_gear != 0 && // Valid ratio
            sensor_data.engine_rpm != 0 // Engine is turning
        ) {
            torque_ratio = ratio_from_c_gear;
            if (ratio_from_t_gear > ratio_from_c_gear) {
                torque_ratio = ratio_from_t_gear;
            }
            torque_ratio *= tcc_multipler;
            torque_ratio *= diff_ratio_f;
            if (torque_ratio < 1) {
                torque_ratio = 1; // HOW!? (diff ratio is always > 2.0)
            }
        } else if (sensor_data.engine_rpm == 0) {
            torque_ratio = -1; // Cannot calculate
        }
        egs_can_hal->set_wheel_torque_multi_factor(torque_ratio);
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
            if (this->flaring && SBS.f_shown_if_flare)
            {
                // Takes president
                egs_can_hal->set_display_msg(GearboxMessage::None);
                egs_can_hal->set_display_gear(GearboxDisplayGear::Failure, false);
            }
            else
            {
                if (this->current_profile == race && this->fwd_gear_shift && SBS.debug_show_up_down_arrows_in_r) {
                    egs_can_hal->set_display_msg(this->is_upshift ? GearboxMessage::Upshift : GearboxMessage::Downshift);
                } else if ((this->current_profile == manual || this->current_profile == race) &&
                    sensor_data.engine_rpm > this->redline_rpm - 1000
                ) {
                    egs_can_hal->set_display_msg(GearboxMessage::Upshift);
                } else {
                     egs_can_hal->set_display_msg(GearboxMessage::None);
                }
                egs_can_hal->set_display_gear(this->current_profile->get_display_gear(this->target_gear, this->actual_gear), this->current_profile == manual);
            }
        }
        portEXIT_CRITICAL(&this->profile_mutex);
        pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);
        vTaskDelay(20 / portTICK_PERIOD_MS); // 50 updates/sec!
    }
}

bool Gearbox::process_speed_sensors()
{
    bool ok = true;
    bool conduct_sanity_check = gear_disagree_count == 0 &&
                                (this->actual_gear == this->target_gear) && (                                                 // Same gear (Not shifting)
                                                                                (this->actual_gear == GearboxGear::Second) || // And in 2..
                                                                                (this->actual_gear == GearboxGear::Third) ||  // .. or 3 ..
                                                                                (this->actual_gear == GearboxGear::Fourth)    // .. or 4
                                                                            );
    uint16_t n2 = TCUIO::n2_rpm();
    uint16_t n3 = TCUIO::n3_rpm();
    uint16_t output = TCUIO::output_rpm();

    if (UINT16_MAX != n2 && UINT16_MAX != n3) {
        uint16_t turbine = TCUIO::calc_turbine_rpm(n2, n3);
        if (conduct_sanity_check) {
            if (abs(n2 - n3) > 100) {
                ok = false;
            }
        }
        if (ok) {
            this->speed_sensors.turbine = turbine;
        }
        this->speed_sensors.n2 = n2;
        this->speed_sensors.n3 = n3;
    }
    
    if (UINT16_MAX != output) {
        speed_sensors.output = output;
    } else {
        ok = false; // Output RPM failed
    }

    return ok;
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
