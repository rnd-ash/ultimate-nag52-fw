#include "diag_data.h"
#include "sensors.h"
#include "solenoids/solenoids.h"
#include "solenoids/constant_current.h"
#include "perf_mon.h"
#include <tcu_maths.h>
#include "kwp2000.h"
#include "esp_core_dump.h"

DATA_GEARBOX_SENSORS get_gearbox_sensors(Gearbox* g) {
    DATA_GEARBOX_SENSORS ret = {};
    if (g == nullptr) {
        memset(&ret, 0xFF, sizeof(ret));
        return ret;
    }
    RpmReading d;
    bool b = false;

    if (Sensors::read_input_rpm(&d, false) == ESP_OK) {
        ret.n2_rpm = d.n2_raw;
        ret.n3_rpm = d.n3_raw;
        ret.calculated_rpm = d.calc_rpm;
    } else {
        ret.n2_rpm = 0xFFFF;
        ret.n3_rpm = 0xFFFF;
        ret.calculated_rpm = 0xFFFF;
    }
    if (Sensors::parking_lock_engaged(&b) == ESP_OK) {
         ret.parking_lock = b;
         ret.atf_temp_c = g->sensor_data.atf_temp;
    } else {
        ret.parking_lock = 0xFF;
        ret.atf_temp_c = 0xFFFF;
    }
    ret.v_batt = Solenoids::get_solenoid_voltage();
    ret.calc_ratio = g->get_gear_ratio();
    return ret;
}

DATA_SOLENOIDS get_solenoid_data(Gearbox* gb_ptr) {
    DATA_SOLENOIDS ret = {};
    if (gb_ptr == nullptr) {
        memset(&ret, 0xFF, sizeof(ret));
        return ret;
    }

    ret.mpc_current = sol_mpc->get_current_avg(); //sol_mpc->get_current_estimate();
    ret.spc_current = sol_spc->get_current_avg();//sol_spc->get_current_estimate();
    ret.tcc_current = sol_tcc->get_current_avg();//sol_tcc->get_current_estimate();
    ret.y3_current = sol_y3->get_current_avg();//sol_y3->get_current_estimate();
    ret.y4_current = sol_y4->get_current_avg();//sol_y4->get_current_estimate();
    ret.y5_current = sol_y5->get_current_avg();//sol_y5->get_current_estimate();
    ret.adjustment_mpc = (mpc_cc->get_adjustment()+1)*1000;
    ret.adjustment_spc = (spc_cc->get_adjustment()+1)*1000;
    ret.mpc_pwm = sol_mpc->get_pwm_compensated();
    ret.spc_pwm = sol_spc->get_pwm_compensated();
    ret.tcc_pwm = sol_tcc->get_pwm_compensated();
    ret.y3_pwm = sol_y3->get_pwm_compensated();
    ret.y4_pwm = sol_y4->get_pwm_compensated();
    ret.y5_pwm = sol_y5->get_pwm_compensated();

    if (gb_ptr->pressure_mgr != nullptr) {
        ret.targ_mpc_current = gb_ptr->pressure_mgr->get_targ_mpc_current();
        ret.targ_spc_current = gb_ptr->pressure_mgr->get_targ_spc_current();
    } else {
        ret.targ_mpc_current = 0xFFFF;
        ret.targ_spc_current = 0xFFFF;
    }
    return ret;
}

DATA_PRESSURES get_pressure_data(Gearbox* gb_ptr) {
    DATA_PRESSURES ret = {};
    if (gb_ptr == nullptr) {
        memset(&ret, 0xFF, sizeof(ret));
        return ret;
    }
    ret.mpc_pwm = sol_mpc->get_pwm_compensated();
    ret.spc_pwm = sol_spc->get_pwm_compensated();
    ret.tcc_pwm = sol_tcc->get_pwm_compensated();
    if (gb_ptr->pressure_mgr == nullptr) {
        ret.mpc_pressure = 0xFFFF;
        ret.spc_pressure = 0xFFFF;
        ret.tcc_pressure = 0xFFFF;
    } else {
        ret.mpc_pressure = gb_ptr->pressure_mgr->get_targ_mpc_pressure();
        ret.spc_pressure = gb_ptr->pressure_mgr->get_targ_spc_pressure();
        ret.tcc_pressure = gb_ptr->pressure_mgr->get_targ_tcc_pressure();
    }
    return ret;
}

DATA_DMA_BUFFER dump_i2s_dma(void) {
    DATA_DMA_BUFFER dma = {};
    dma.dma = 0;
    dma.adc_reading = sol_spc->diag_get_adc_peak_raw();
    return dma;
}

DATA_CANBUS_RX get_rx_can_data(EgsBaseCan* can_layer) {
    DATA_CANBUS_RX ret = {};
    if (can_layer == nullptr || gearbox == nullptr) {
        memset(&ret, 0xFF, sizeof(ret));
        return ret;
    }
    uint64_t now = esp_timer_get_time() / 1000;

    WheelData t = gearbox->sensor_data.rl_wheel;
    ret.left_rear_rpm = t.current_dir == WheelDirection::SignalNotAvaliable ? 0xFFFF : t.double_rpm;
    t = gearbox->sensor_data.rr_wheel;
    ret.right_rear_rpm = t.current_dir == WheelDirection::SignalNotAvaliable ? 0xFFFF : t.double_rpm;

    ret.paddle_position = can_layer->get_paddle_position(now, 250);
    ret.pedal_pos = can_layer->get_pedal_value(now, 250);

    int torque = 0xFFFF;
    torque = can_layer->get_maximum_engine_torque(now, 250);
    ret.max_torque = torque == INT_MAX ? 0xFFFF : (torque + 500)*4;
    torque = can_layer->get_minimum_engine_torque(now, 250);
    ret.min_torque = torque == INT_MAX ? 0xFFFF : (torque + 500)*4;
    torque = can_layer->get_driver_engine_torque(now, 250);
    ret.driver_torque = torque == INT_MAX ? 0xFFFF : (torque + 500)*4;
    torque = can_layer->get_static_engine_torque(now, 250);
    ret.static_torque = torque == INT_MAX ? 0xFFFF : (torque + 500)*4;
    ret.shift_button_pressed = can_layer->get_profile_btn_press(now, 250);
    ret.shifter_position = can_layer->get_shifter_position_ewm(now, 250);
    ret.engine_rpm = can_layer->get_engine_rpm(now, 250);
    ret.fuel_rate = can_layer->get_fuel_flow_rate(now, 250);
    return ret;
}

DATA_SYS_USAGE get_sys_usage(void) {
    DATA_SYS_USAGE ret = {};
    CpuStats s = get_cpu_stats();
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

    ret.spc_pressure = g->pressure_mgr->get_targ_spc_pressure();
    ret.mpc_pressure = g->pressure_mgr->get_targ_mpc_pressure();
    ret.tcc_pressure = g->pressure_mgr->get_targ_tcc_pressure();
    // Hack. As we can guarantee only one solenoid will be on, we can do a fast bitwise OR on all 3 to get the application state
    ret.ss_pos = (sol_y3->get_pwm_raw() | sol_y4->get_pwm_raw() | sol_y5->get_pwm_raw()) >> 8;

    ret.input_rpm = g->sensor_data.input_rpm;
    ret.engine_rpm = g->sensor_data.engine_rpm;
    ret.output_rpm = g->sensor_data.output_rpm;
    ret.engine_torque = g->sensor_data.static_torque;
    ret.atf_temp = g->sensor_data.atf_temp+40;

    if (g->isShifting()) {
        switch(g->get_curr_gear_change()) {
            case ProfileGearChange::ONE_TWO:
                ret.shift_idx = 1;
                break;
            case ProfileGearChange::TWO_THREE:
                ret.shift_idx = 2;
                break;
            case ProfileGearChange::THREE_FOUR:
                ret.shift_idx = 3;
                break;
            case ProfileGearChange::FOUR_FIVE:
                ret.shift_idx = 4;
                break;
            case ProfileGearChange::FIVE_FOUR:
                ret.shift_idx = 5;
                break;
            case ProfileGearChange::FOUR_THREE:
                ret.shift_idx = 6;
                break;
            case ProfileGearChange::THREE_TWO:
                ret.shift_idx = 7;
                break;
            case ProfileGearChange::TWO_ONE:
                ret.shift_idx = 8;
                break;
            default:
                ret.shift_idx = 0xFF;
                break;
        }
    } else {
        ret.shift_idx = 0;
    }
    return ret;   
}


TCM_CORE_CONFIG get_tcm_config(void) {
    TCM_CORE_CONFIG cfg;
    memcpy(&cfg, &VEHICLE_CONFIG, sizeof(TCM_CORE_CONFIG));
    return cfg;
}

uint8_t set_tcm_config(TCM_CORE_CONFIG cfg) {
    ShifterPosition pos = egs_can_hal == nullptr ? ShifterPosition::SignalNotAvaliable : egs_can_hal->get_shifter_position_ewm(esp_timer_get_time()/1000, 250);
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
    if (EEPROM::save_core_config(&cfg)) {
        return 0x00; // OK!
    } else {
        return NRC_GENERAL_REJECT; // SCN write error
    }
}

COREDUMP_INFO get_coredump_info(void) {
    size_t addr = 0;
    size_t size = 0;
    esp_core_dump_image_get(&addr, &size);
    return COREDUMP_INFO {
        .address = addr,
        .size = size
    };
}

const esp_app_desc_t* get_image_header(void) {
    return esp_ota_get_app_description();
}