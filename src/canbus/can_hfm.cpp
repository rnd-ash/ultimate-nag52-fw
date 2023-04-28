#include "can_hfm.h"
#include "driver/twai.h"
#include "driver/i2c.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"

#include "shifter/shifter_ewm.h"
#include "shifter/shifter_trrs.h"

HfmCan::HfmCan(const char* name, uint8_t tx_time_ms, uint32_t baud) : EgsBaseCan(name, tx_time_ms, baud) {
    ESP_LOGI("HfmCan", "SETUP CALLED");
}

WheelData HfmCan::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData HfmCan::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData HfmCan::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData HfmCan::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

ShifterPosition HfmCan::get_shifter_position(uint64_t now, uint64_t expire_time_ms) {
    return shifter->get_shifter_position(now, expire_time_ms);
}

EngineType HfmCan::get_engine_type(uint64_t now, uint64_t expire_time_ms) {
    return EngineType::Unknown;
}

bool HfmCan::get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool HfmCan::get_kickdown(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

uint8_t HfmCan::get_pedal_value(uint64_t now, uint64_t expire_time_ms) { // TODO
    return 0xFF;
}

int HfmCan::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    return INT_MAX;
}

int HfmCan::get_driver_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    return this->get_static_engine_torque(now, expire_time_ms);
}

int HfmCan::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    return INT_MAX;
}

int HfmCan::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    return INT_MAX;
}

PaddlePosition HfmCan::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    return PaddlePosition::SNV;
}

int16_t HfmCan::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    return INT16_MAX;
}

int16_t HfmCan::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) {
    return INT16_MAX;
}

int16_t HfmCan::get_engine_iat_temp(uint64_t now, uint64_t expire_time_ms) {
    return INT16_MAX;
}

uint16_t HfmCan::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    return UINT16_MAX;
}

bool HfmCan::get_is_starting(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool HfmCan::get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

bool HfmCan::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

void HfmCan::set_clutch_status(ClutchStatus status) {
    
}

void HfmCan::set_actual_gear(GearboxGear actual) {
}

void HfmCan::set_target_gear(GearboxGear target) {
}

void HfmCan::set_safe_start(bool can_start) {
}

void HfmCan::set_gearbox_temperature(uint16_t temp) {
}

void HfmCan::set_input_shaft_speed(uint16_t rpm) {
}

void HfmCan::set_is_all_wheel_drive(bool is_4wd) {
}

void HfmCan::set_wheel_torque(uint16_t t) {
}

void HfmCan::set_shifter_position(ShifterPosition pos) {
}

void HfmCan::set_gearbox_ok(bool is_ok) {
}

void HfmCan::set_torque_request(TorqueRequest request, float amount_nm) {
}

void HfmCan::set_error_check_status(SystemStatusCheck ssc) {
}

void HfmCan::set_turbine_torque_loss(uint16_t loss_nm) {
}

void HfmCan::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
}

void HfmCan::set_drive_profile(GearboxProfile p) {
}

void HfmCan::set_display_msg(GearboxMessage msg) {
}

void HfmCan::set_wheel_torque_multi_factor(float ratio) {
}

void HfmCan::tx_frames() {
    // None to transmit on HFM!
}

void HfmCan::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) {
    this->hfm_ecu.import_frames(data, id, timestamp);
}

void HfmCan::on_rx_done(uint64_t now_ts) {
}