#include "can_custom.h"

#include "driver/twai.h"
#include "driver/i2c_master.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"
#include "shifter/shifter_trrs.h"
#include "shifter/shifter_ewm.h"

CustomCan::CustomCan(const char *name, uint8_t tx_time_ms, uint32_t baud) : EgsBaseCan(name, tx_time_ms, baud) 
{
    ESP_LOGI("CustomCAN", "SETUP CALLED");
}

uint16_t CustomCan::get_front_right_wheel(const uint32_t expire_time_ms)
{
	WHEELS_300_CUSTOMCAN wheel_data{};
    uint16_t ret = UINT16_MAX;
    if (this->wheels.get_WHEELS_300(GET_CLOCK_TIME(), expire_time_ms, &wheel_data)) {
        ret = wheel_data.RPM_2X_FR;
    }
    return ret;
}

uint16_t CustomCan::get_front_left_wheel(const uint32_t expire_time_ms) { // TODO
    WHEELS_300_CUSTOMCAN wheel_data{};
    uint16_t ret = UINT16_MAX;
    if (this->wheels.get_WHEELS_300(GET_CLOCK_TIME(), expire_time_ms, &wheel_data)) {
        ret = wheel_data.RPM_2X_FL;
    }
    return ret;
}

uint16_t CustomCan::get_rear_right_wheel(const uint32_t expire_time_ms) {
    WHEELS_300_CUSTOMCAN wheel_data{};
    uint16_t ret = UINT16_MAX;
    if (this->wheels.get_WHEELS_300(GET_CLOCK_TIME(), expire_time_ms, &wheel_data)) {
        ret = wheel_data.RPM_2X_RR;
    }
    return ret;
}

uint16_t CustomCan::get_rear_left_wheel(const uint32_t expire_time_ms) {
    WHEELS_300_CUSTOMCAN wheel_data{};
    uint16_t ret = UINT16_MAX;
    if (this->wheels.get_WHEELS_300(GET_CLOCK_TIME(), expire_time_ms, &wheel_data)) {
        ret = wheel_data.RPM_2X_RL;
    }
    return ret;
}

EngineType CustomCan::get_engine_type(const uint32_t expire_time_ms) {
    return EngineType::Unknown;
}

bool CustomCan::get_engine_is_limp(const uint32_t expire_time_ms) { // TODO
    return false;
}

bool CustomCan::get_kickdown(const uint32_t expire_time_ms) { // TODO
    return false;
}

uint8_t CustomCan::get_pedal_value(const uint32_t expire_time_ms) {
    ENGINE_100_CUSTOMCAN engine_data{};
    if (this->engine.get_ENGINE_100(GET_CLOCK_TIME(), expire_time_ms, &engine_data)) {
        return engine_data.PEDAL;
    } else {
        return 0xFF;
    }
}

CanTorqueData CustomCan::get_torque_data(const uint32_t expire_time_ms) {
    ENGINE_102_CUSTOMCAN torque_data{};
    CanTorqueData ret = TORQUE_NDEF;
    if (this->engine.get_ENGINE_102(GET_CLOCK_TIME(), expire_time_ms, &torque_data)) {
        if (UINT16_MAX != torque_data.STATIC_TORQUE) {
            ret.m_converted_static = ((int)torque_data.STATIC_TORQUE / 4) - 500;
        }
        if (UINT16_MAX != torque_data.DRIVER_TORQUE) {
            ret.m_converted_driver = ((int)torque_data.DRIVER_TORQUE / 4) - 500;
        }
        if (UINT16_MAX != torque_data.MIN_TORQUE) {
            ret.m_min = ((int)torque_data.MIN_TORQUE / 4) - 500;
        }
        if (UINT16_MAX != torque_data.MAX_TORQUE) {
            ret.m_max = ((int)torque_data.MAX_TORQUE / 4) - 500;
        }
        ret.m_ind = ret.m_converted_driver;
    }
    return ret;
}

PaddlePosition CustomCan::get_paddle_position(const uint32_t expire_time_ms) {
    return PaddlePosition::SNV;
}

int16_t CustomCan::get_engine_coolant_temp(const uint32_t expire_time_ms) {
    ENGINE_100_CUSTOMCAN engine_data{};
    int16_t ret = INT16_MAX;
    if (this->engine.get_ENGINE_100(GET_CLOCK_TIME(), expire_time_ms, &engine_data)) {
        if (engine_data.T_COOLANT != UINT8_MAX) {
            ret = (int16_t)engine_data.PEDAL - 40;
        }
    }
    return ret;
}

int16_t CustomCan::get_engine_oil_temp(const uint32_t expire_time_ms) {
    ENGINE_100_CUSTOMCAN engine_data{};
    int16_t ret = INT16_MAX;
    if (this->engine.get_ENGINE_100(GET_CLOCK_TIME(), expire_time_ms, &engine_data)) {
        if (engine_data.T_OIL != UINT8_MAX) {
            ret = (int16_t)engine_data.T_OIL - 40;
        }
    }
    return ret;
}

int16_t CustomCan::get_engine_iat_temp(const uint32_t expire_time_ms) {
    return INT16_MAX;
}

uint16_t CustomCan::get_engine_rpm(const uint32_t expire_time_ms) {
    ENGINE_100_CUSTOMCAN engine_data{};
    uint16_t ret = UINT16_MAX;
    if (this->engine.get_ENGINE_100(GET_CLOCK_TIME(), expire_time_ms, &engine_data)) {
        ret = engine_data.RPM;
    }
    return ret;
}

bool CustomCan::get_is_starting(const uint32_t expire_time_ms) { // TODO
    return false;
}

bool CustomCan::get_is_brake_pressed(const uint32_t expire_time_ms) {
    return false;
}

bool CustomCan::get_profile_btn_press(const uint32_t expire_time_ms) {
    return false;
}

uint16_t CustomCan::get_fuel_flow_rate(const uint32_t expire_time_ms) {
    return  0;
}

TccReqState CustomCan::get_engine_tcc_override_request(const uint32_t expire_time_ms) {
    return TccReqState::None;
}

void CustomCan::set_clutch_status(TccClutchStatus status) {
}

void CustomCan::set_actual_gear(GearboxGear actual) {
    switch(actual) {
        case GearboxGear::First:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::D1;
            break;
        case GearboxGear::Second:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::D2;
            break;
        case GearboxGear::Third:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::D3;
            break;
        case GearboxGear::Fourth:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::D4;
            break;
        case GearboxGear::Fifth:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::D5;
            break;
        case GearboxGear::Park:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::P;
            break;
        case GearboxGear::Neutral:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::N;
            break;
        case GearboxGear::Reverse_First:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::R1;
            break;
        case GearboxGear::Reverse_Second:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::R2;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->tx_400.A_GEAR = UN52_400h_A_GEAR_CUSTOMCAN::SNV;
            break;
    }
}

void CustomCan::set_target_gear(GearboxGear target) {
    switch(target) {
        case GearboxGear::First:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::D1;
            break;
        case GearboxGear::Second:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::D2;
            break;
        case GearboxGear::Third:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::D3;
            break;
        case GearboxGear::Fourth:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::D4;
            break;
        case GearboxGear::Fifth:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::D5;
            break;
        case GearboxGear::Park:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::P;
            break;
        case GearboxGear::Neutral:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::N;
            break;
        case GearboxGear::Reverse_First:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::R1;
            break;
        case GearboxGear::Reverse_Second:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::R2;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->tx_400.T_GEAR = UN52_400h_T_GEAR_CUSTOMCAN::SNV;
            break;
    }
}

void CustomCan::set_gearbox_temperature(int16_t temp) {
    this->tx_400.T_OEL = (MAX(temp, -50) + 50) & 0xFF;
}

void CustomCan::set_input_shaft_speed(uint16_t rpm) {
}

void CustomCan::set_is_all_wheel_drive(bool is_4wd) {
}

void CustomCan::set_wheel_torque(uint16_t t) {
}

void CustomCan::set_shifter_position(ShifterPosition pos) {
}

void CustomCan::set_gearbox_ok(bool is_ok) {
    this->tx_400.GB_OK = is_ok;
}

void CustomCan::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    if (control_type == TorqueRequestControlType::None) {
        tx_410.TRQ_REQ_CTRL0 = false;
        tx_410.TRQ_REQ_CTRL1 = false;
    } else if (control_type == TorqueRequestControlType::FastAsPossible) {
        tx_410.TRQ_REQ_CTRL0 = false;
        tx_410.TRQ_REQ_CTRL1 = true;
    } else if (control_type == TorqueRequestControlType::BackToDemandTorque) {
        tx_410.TRQ_REQ_CTRL0 = true;
        tx_410.TRQ_REQ_CTRL1 = true;
    } else { // Normal speed
        tx_410.TRQ_REQ_CTRL0 = true;
        tx_410.TRQ_REQ_CTRL1 = false;
    }
    if (control_type != TorqueRequestControlType::None) {
        tx_410.TRQ_REQ_TRQ = (amount_nm + 500) * 4;
        if (limit_type == TorqueRequestBounds::LessThan) {
            tx_410.TRQ_REQ_MIN = true;
            tx_410.TRQ_REQ_MAX = false;
        } else if (limit_type == TorqueRequestBounds::MoreThan) {
            tx_410.TRQ_REQ_MIN = false;
            tx_410.TRQ_REQ_MAX = true;
        } else {
            tx_410.TRQ_REQ_MIN = true;
            tx_410.TRQ_REQ_MAX = true;
        }
    } else {
        tx_410.TRQ_REQ_MIN = false;
        tx_410.TRQ_REQ_MAX = false;
        tx_410.TRQ_REQ_TRQ = 0;
    }
}

void CustomCan::set_garage_shift_state(bool enable) {
    this->tx_400.G_SHIFT = enable;
}

void CustomCan::set_error_check_status(SystemStatusCheck ssc) {
}

void CustomCan::set_turbine_torque_loss(uint16_t loss_nm) {
}

void CustomCan::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
}

void CustomCan::set_drive_profile(GearboxProfile p) {
}

void CustomCan::set_display_msg(GearboxMessage msg) {
}

void CustomCan::set_wheel_torque_multi_factor(float ratio) {
}

void CustomCan::tx_frames() {
    tx.data_length_code = 8;
    UN52_400_CUSTOMCAN tx400;
    tx400 = {this->tx_400.raw};

    UN52_410_CUSTOMCAN tx410;
    tx410 = {this->tx_410.raw};

    tx410.TRQ_REQ_TOGGLE = this->toggle_bit;
    this->toggle_bit = !this->toggle_bit;

    tx.identifier = UN52_400_CUSTOMCAN_CAN_ID;
    to_bytes(tx400.raw, tx.data);
    twai_transmit(&tx, 5);

    tx.identifier = UN52_410_CUSTOMCAN_CAN_ID;
    to_bytes(tx410.raw, tx.data);
    twai_transmit(&tx, 5);
}

void CustomCan::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) {
    if (this->engine.import_frames(data, id, timestamp)) {
    } else if (this->wheels.import_frames(data, id, timestamp)) {
    } else if (this->brakes.import_frames(data, id, timestamp)) {
    } else if (this->ewm.import_frames(data, id, timestamp)) {
    }
}