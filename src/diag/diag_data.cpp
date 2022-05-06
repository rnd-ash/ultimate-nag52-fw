#include "diag_data.h"
#include "sensors.h"
#include "solenoids/solenoids.h"
#include "perf_mon.h"
#include <tcm_maths.h>
#include "kwp2000.h"
#include "esp_core_dump.h"

DATA_GEARBOX_SENSORS get_gearbox_sensors(Gearbox* g) {
    DATA_GEARBOX_SENSORS ret = {};
    RpmReading d;
    bool b = false;
    if (!Sensors::read_input_rpm(&d, false)) {
        ret.n2_rpm = 0xFFFF;
        ret.n3_rpm = 0xFFFF;
        ret.calculated_rpm = 0xFFFF;
    } else {
        ret.n2_rpm = d.n2_raw;
        ret.n3_rpm = d.n3_raw;
        ret.calculated_rpm = d.calc_rpm;
    }
    if (Sensors::parking_lock_engaged(&b)) {
         ret.parking_lock = b;
         ret.atf_temp_c = g->sensor_data.atf_temp;
    } else {
        ret.parking_lock = 0xFF;
        ret.atf_temp_c = 0xFFFF;
    }
    if (!Sensors::read_vbatt(&ret.v_batt)) {
        ret.v_batt = 0xFFFF;
    }
    ret.calc_ratio = g->get_gear_ratio();
    return ret;
}

DATA_SOLENOIDS get_solenoid_data() {
    DATA_SOLENOIDS ret = {};

    ret.mpc_current = sol_mpc->get_current_estimate();
    ret.spc_current = sol_spc->get_current_estimate();
    ret.tcc_current = sol_tcc->get_current_estimate();
    ret.y3_current = sol_y3->get_current_estimate();
    ret.y4_current = sol_y4->get_current_estimate();
    ret.y5_current = sol_y5->get_current_estimate();

    ret.mpc_pwm = sol_mpc->get_pwm();
    ret.spc_pwm = sol_spc->get_pwm();
    ret.tcc_pwm = sol_tcc->get_pwm();
    ret.y3_pwm = sol_y3->get_pwm();
    ret.y4_pwm = sol_y4->get_pwm();
    ret.y5_pwm = sol_y5->get_pwm();

    return ret;
}

DATA_CANBUS_RX get_rx_can_data(AbstractCan* can_layer) {
    DATA_CANBUS_RX ret = {};
    uint64_t now = esp_timer_get_time() / 1000;

    WheelData t = can_layer->get_rear_left_wheel(now, 250);
    ret.left_rear_rpm = t.current_dir == WheelDirection::SignalNotAvaliable ? 0xFFFF : t.double_rpm;
    t = can_layer->get_rear_right_wheel(now, 250);
    ret.right_rear_rpm = t.current_dir == WheelDirection::SignalNotAvaliable ? 0xFFFF : t.double_rpm;

    ret.paddle_position = can_layer->get_paddle_position(now, 250);
    ret.pedal_pos = can_layer->get_pedal_value(now, 250);

    int torque = 0xFFFF;
    torque = can_layer->get_maximum_engine_torque(now, 250);
    ret.max_torque = torque == INT_MAX ? 0xFFFF : (torque + 500)*4;
    torque = can_layer->get_minimum_engine_torque(now, 250);
    ret.min_torque = torque == INT_MAX ? 0xFFFF : (torque + 500)*4;
    torque = can_layer->get_static_engine_torque(now, 250);
    ret.static_torque = torque == INT_MAX ? 0xFFFF : (torque + 500)*4;
    ret.shift_button_pressed = can_layer->get_profile_btn_press(now, 250);
    ret.shifter_position = can_layer->get_shifter_position_ewm(now, 250);
    return ret;
}

DATA_SYS_USAGE get_sys_usage() {
    DATA_SYS_USAGE ret = {};

    ret.free_heap = esp_get_free_heap_size();
    CpuStats s = get_cpu_usage();
    ret.core1_usage = s.load_core_1;
    ret.core2_usage = s.load_core_2;
    ret.num_tasks = uxTaskGetNumberOfTasks();
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
    ret.free_psram = info.total_free_bytes;
    return ret;
}


TCM_CORE_CONFIG get_tcm_config() {
    TCM_CORE_CONFIG cfg;
    memcpy(&cfg, &VEHICLE_CONFIG, sizeof(TCM_CORE_CONFIG));
    return cfg;
}

uint8_t set_tcm_config(TCM_CORE_CONFIG cfg) {
    ShifterPosition pos = egs_can_hal->get_shifter_position_ewm(esp_timer_get_time()/1000, 250);
    if (
        pos == ShifterPosition::D || pos == ShifterPosition::MINUS || pos == ShifterPosition::PLUS || pos == ShifterPosition::R || // Stationary positions
        pos == ShifterPosition::N_D || pos == ShifterPosition::P_R || pos == ShifterPosition::R_N // Intermediate positions
        ) {
            ESP_LOGE("SET_TCM_CFG", "Rejecting download request. Shifter not in valid position");
            return NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR;
    }
    // S,C,W,A,M = 5 profiles, so 4 is max value
    if (cfg.default_profile > 4) {
        ESP_LOGE("SET_TCM_CFG", "Default profile ID of %d was greater than 4", cfg.default_profile);
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
    if (cfg.engine_type != 0 && cfg.engine_type != 1) {
        ESP_LOGE("SET_TCM_CFG", "Engine type was not 0 (Diesel) or 1 (Petrol)");
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
    if (cfg.is_four_matic && (cfg.transfer_case_high_ratio == 0 || cfg.transfer_case_low_ratio == 0)) {
        ESP_LOGE("SET_TCM_CFG", "4Matic was requested, but TC ratio was 0");
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
    if (EEPROM::save_core_config(&cfg)) {
        return 0x00; // OK!
    } else {
        return NRC_GENERAL_REJECT; // SCN write error
    }
}

COREDUMP_INFO get_coredump_info() {
    size_t addr = 0;
    size_t size = 0;
    esp_core_dump_image_get(&addr, &size);
    return COREDUMP_INFO {
        .address = addr,
        .size = size
    };
}