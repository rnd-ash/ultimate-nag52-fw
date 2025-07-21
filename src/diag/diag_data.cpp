#include "diag_data.h"
#include "sensors.h"
#include "solenoids/solenoids.h"
#include "perf_mon.h"
#include <tcu_maths.h>
#include "kwp2000.h"
#include "esp_core_dump.h"
#include "../nvs/module_settings.h"
#include "clock.hpp"
#include "egs_calibration/calibration_structs.h"
#include "tcu_io/tcu_io.hpp"
#include "../embed_data.h"

DATA_GEARBOX_SENSORS get_gearbox_sensors(Gearbox* g) {
    DATA_GEARBOX_SENSORS ret = {};
    SpeedSensors speeds = gearbox->speed_sensors;
    uint8_t pll = TCUIO::parking_lock();

    ret.n2_rpm = speeds.n2;
    ret.n3_rpm = speeds.n3;
    ret.calculated_rpm = speeds.turbine;
    
    if (UINT8_MAX != pll) {
        ret.parking_lock = pll;
        if (pll == 0) {
            int16_t tft = TCUIO::atf_temperature();
            ret.atf_temp_c = tft;
        }
    } else {
        ret.parking_lock = 0xFF;
        ret.atf_temp_c = 0xFFFF;
    }
    ret.v_batt = Solenoids::get_solenoid_voltage();
    if (g == nullptr) {
        ret.calc_ratio = UINT16_MAX;
        ret.targ_ratio = UINT16_MAX;
    } else {
        ret.calc_ratio = g->get_gear_ratio();
        ret.targ_ratio = g->get_targ_gear_ratio();
    }
    ret.output_rpm = speeds.output;
    return ret;
}

DATA_SOLENOIDS get_solenoid_data(Gearbox* gb_ptr) {
    DATA_SOLENOIDS ret = {};
    if (sol_mpc == nullptr) {
        memset(&ret, 0xFF, sizeof(ret));
        return ret;
    }

    ret.mpc_current = sol_mpc->get_current() & 0xFFFF; //sol_mpc->get_current_estimate();
    ret.spc_current = sol_spc->get_current() & 0xFFFF;//sol_spc->get_current_estimate();
    ret.tcc_current = sol_tcc->get_current() & 0xFFFF;//sol_tcc->get_current_estimate();
    ret.y3_current = sol_y3->get_current() & 0xFFFF;//sol_y3->get_current_estimate();
    ret.y4_current = sol_y4->get_current() & 0xFFFF;//sol_y4->get_current_estimate();
    ret.y5_current = sol_y5->get_current() & 0xFFFF;//sol_y5->get_current_estimate();
    ret.adjustment_mpc = (uint16_t)(sol_mpc->get_trim()*1000) & 0xFFFF;
    ret.adjustment_spc = (uint16_t)(sol_spc->get_trim()*1000) & 0xFFFF;
    ret.mpc_pwm = sol_mpc->get_pwm_compensated();
    ret.spc_pwm = sol_spc->get_pwm_compensated();
    ret.tcc_pwm = sol_tcc->get_pwm_compensated();
    ret.y3_pwm = sol_y3->get_pwm_compensated();
    ret.y4_pwm = sol_y4->get_pwm_compensated();
    ret.y5_pwm = sol_y5->get_pwm_compensated();
    ret.targ_mpc_current = sol_mpc->get_current_target();
    ret.targ_spc_current = sol_spc->get_current_target();
    
    return ret;
}

DATA_PRESSURES get_pressure_data(Gearbox* gb_ptr) {
    DATA_PRESSURES ret = {};
    memset(&ret, 0xFF, sizeof(ret));
    if (gb_ptr == nullptr) {
        return ret;
    }
    if (nullptr != gb_ptr->pressure_mgr) {
        ret.shift_req_pressure = gb_ptr->pressure_mgr->get_input_shift_pressure();
        ret.modulating_req_pressure = gb_ptr->pressure_mgr->get_input_modulating_pressure();
        ret.working_pressure = gb_ptr->pressure_mgr->get_calc_line_pressure();
        ret.inlet_pressure = gb_ptr->pressure_mgr->get_calc_inlet_pressure();
        ret.corrected_spc_pressure = gb_ptr->pressure_mgr->get_corrected_spc_pressure();
        ret.corrected_mpc_pressure = gb_ptr->pressure_mgr->get_corrected_modulating_pressure();
        ret.tcc_pressure = gb_ptr->pressure_mgr->get_targ_tcc_pressure();
        ret.ss_flag = gb_ptr->pressure_mgr->get_active_shift_circuits();
        ShiftPressures p = gb_ptr->pressure_mgr->get_shift_pressures_now();
        ret.overlap_mod = p.overlap_mod;
        ret.overlap_shift = p.on_clutch; //p.overlap_shift;
        ret.on_clutch_pressure = p.on_clutch;
        ret.off_clutch_pressure = p.off_clutch;
    }
    return ret;
}

DATA_TCC_PROGRAM get_tcc_program_data(Gearbox* gb_ptr) {
    DATA_TCC_PROGRAM ret = {};
    ret.current_pressure = gb_ptr->tcc->get_current_pressure();
    ret.target_pressure = gb_ptr->tcc->get_target_pressure();
    ret.slip_filtered = gb_ptr->tcc->get_slip_filtered();
    ret.slip_now = (int16_t)gb_ptr->sensor_data.engine_rpm - (int16_t)gb_ptr->sensor_data.input_rpm;
    ret.pedal_filtered = gb_ptr->sensor_data.pedal_smoothed->get_average();
    ret.pedal_now = gb_ptr->sensor_data.pedal_pos;
    ret.slip_target = gb_ptr->tcc->get_slip_targ();
    ret.targ_state = gb_ptr->tcc->get_target_state();
    ret.current_state = gb_ptr->tcc->get_current_state();
    ret.can_request_bits = 0; // TODO
    ret.tcc_absorbed_joule = gb_ptr->tcc->get_absorbed_power();
    ret.engine_output_joule = gb_ptr->tcc->get_engine_power();
    return ret;
}

DATA_CANBUS_RX get_rx_can_data(EgsBaseCan* can_layer) {
    DATA_CANBUS_RX ret = {};
    if (can_layer == nullptr || gearbox == nullptr) {
        memset(&ret, 0xFF, sizeof(ret));
        return ret;
    }
    
    ret.left_rear_rpm = TCUIO::wheel_rl_2x_rpm();
    if (UINT16_MAX != ret.left_rear_rpm) {
        ret.left_rear_rpm /= 2;
    }
    ret.right_rear_rpm = TCUIO::wheel_rr_2x_rpm();
    if (UINT16_MAX != ret.right_rear_rpm) {
        ret.right_rear_rpm /= 2;
    }

    ret.paddle_position = can_layer->get_paddle_position(250);
    ret.pedal_pos = can_layer->get_pedal_value(250);

    int torque = 0xFFFF;
    torque = gearbox->sensor_data.max_torque;
    ret.max_torque = (torque+500)*4;
    torque = gearbox->sensor_data.min_torque;
    ret.min_torque = (torque+500)*4;
    ret.driver_torque = (gearbox->sensor_data.converted_driver_torque+500)*4;
    ret.static_torque = (gearbox->sensor_data.converted_torque+500)*4;
    ret.profile_input_raw = can_layer->shifter->diag_get_profile_input();
    ret.shifter_position = can_layer->get_shifter_position(250);
    ret.engine_rpm = can_layer->get_engine_rpm(250);
    ret.fuel_rate = can_layer->get_fuel_flow_rate(250);
    ret.torque_req_ctrl_type = gearbox->output_data.ctrl_type;
    ret.torque_req_bounds = gearbox->output_data.bounds;
    ret.torque_req_amount = ret.torque_req_ctrl_type == TorqueRequestControlType::None ? 0xFFFF : (gearbox->output_data.torque_req_amount+500)*4;
    // Temps
    ret.e_coolant_temp = egs_can_hal->get_engine_coolant_temp(250);
    ret.e_iat_temp = egs_can_hal->get_engine_iat_temp(250);
    ret.e_oil_temp = egs_can_hal->get_engine_oil_temp(250);
    return ret;
}

DATA_SYS_USAGE get_sys_usage(void) {
    DATA_SYS_USAGE ret = {};
    CpuStats s = PerfMon::get_cpu_stats();
    ret.core1_usage = s.load_core_1;
    ret.core2_usage = s.load_core_2;
    ret.num_tasks = uxTaskGetNumberOfTasks();
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
    ret.free_psram = info.total_free_bytes;
    ret.total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);
    ret.free_ram = info.total_free_bytes;
    ret.total_ram = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    return ret;
}

SHIFT_LIVE_INFO get_shift_live_Data(const EgsBaseCan* can_layer, Gearbox* g) {
    SHIFT_LIVE_INFO ret = {};
    if (can_layer == nullptr || g == nullptr) {
        memset(&ret, 0xFF, sizeof(ret));
        return ret;
    }

    ret.spc_pressure = g->pressure_mgr->get_corrected_spc_pressure();
    ret.mpc_pressure = g->pressure_mgr->get_corrected_modulating_pressure();
    ret.tcc_pressure = g->pressure_mgr->get_targ_tcc_pressure();
    // Hack. As we can guarantee only one solenoid will be on, we can do a fast bitwise OR on all 3 to get the application state
    ret.ss_pos = (sol_y3->get_pwm_raw() | sol_y4->get_pwm_raw() | sol_y5->get_pwm_raw()) >> 8;

    ret.input_rpm = g->sensor_data.input_rpm;
    ret.engine_rpm = g->sensor_data.engine_rpm;
    ret.output_rpm = g->sensor_data.output_rpm;
    ret.engine_torque = g->sensor_data.converted_driver_torque;
    ret.input_torque = g->sensor_data.input_torque;
    ret.req_engine_torque = g->output_data.ctrl_type == TorqueRequestControlType::None ? INT16_MAX : g->output_data.torque_req_amount;
    ret.atf_temp = g->sensor_data.atf_temp+40;
    ret.profile = g->get_profile_id();
    ret.targ_act_gear = g->get_targ_curr_gear();
    return ret;   
}


TCM_CORE_CONFIG get_tcm_config(void) {
    TCM_CORE_CONFIG cfg;
    memcpy(&cfg, &VEHICLE_CONFIG, sizeof(TCM_CORE_CONFIG));
    return cfg;
}

kwp_result_t set_tcm_config(TCM_CORE_CONFIG cfg) {
    ShifterPosition pos = (egs_can_hal == nullptr || gearbox == nullptr) ? ShifterPosition::SignalNotAvailable : egs_can_hal->get_shifter_position(250);
    if (
        pos == ShifterPosition::D || pos == ShifterPosition::MINUS || pos == ShifterPosition::PLUS || pos == ShifterPosition::R || // Stationary positions
        pos == ShifterPosition::N_D || pos == ShifterPosition::P_R || pos == ShifterPosition::R_N // Intermediate positions
        ) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "SET_TCM_CFG", "Rejecting download request. Shifter not in valid position");
            return NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR;
    }
    // S,C,W,A,M = 5 profiles, so 4 is max value
    if (cfg.default_profile > 4) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SET_TCM_CFG", "Default profile ID of %d was greater than 4", cfg.default_profile);
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
    if (cfg.engine_type != 0 && cfg.engine_type != 1) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SET_TCM_CFG", "Engine type was not 0 (Diesel) or 1 (Petrol)");
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
    if (cfg.is_four_matic && (cfg.transfer_case_high_ratio == 0 || cfg.transfer_case_low_ratio == 0)) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SET_TCM_CFG", "4Matic was requested, but TC ratio was 0");
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
    if (EEPROM::save_core_config(&cfg) == ESP_OK) {
        return 0x00; // OK!
    } else {
        return NRC_GENERAL_REJECT; // SCN write error
    }
}

PARTITION_INFO get_coredump_info(void) {
    size_t addr = 0;
    size_t size = 0;
    esp_core_dump_image_get(&addr, &size);
    return PARTITION_INFO {
        .address = addr,
        .size = size
    };
}

PARTITION_INFO get_current_sw_info(void) {
    const esp_partition_t* i = esp_ota_get_running_partition();
    return PARTITION_INFO {
        .address = i->address,
        .size = i->size
    };
}

PARTITION_INFO get_next_sw_info(void) {
    const esp_partition_t* i = esp_ota_get_next_update_partition(NULL);
    return PARTITION_INFO {
        .address = i->address,
        .size = i->size
    };
}

PARTITION_INFO get_embeded_file_info(void) {
    uint32_t len = (uint32_t)embed_container_end - (uint32_t)embed_container_start;
    return PARTITION_INFO {
        .address = (uint32_t)embed_container_start,
        .size = len
    };
}

const esp_app_desc_t* get_image_header(void) {
    return esp_app_get_description();
}

uint16_t get_egs_calibration_size(void) {
    return sizeof(CalibrationInfo);
}

kwp_result_t get_module_settings(uint8_t module_id, uint16_t* buffer_len, uint8_t** buffer) {
    return ModuleConfiguration::read_settings(module_id, buffer_len, buffer);
}

kwp_result_t set_module_settings(uint8_t module_id, uint16_t buffer_len, uint8_t* buffer) {
    if (buffer_len == 1 && buffer[0] == 0x00) {
        return ModuleConfiguration::reset_settings(module_id);
    } else {
        return ModuleConfiguration::write_settings(module_id, buffer_len, buffer);
    }
}