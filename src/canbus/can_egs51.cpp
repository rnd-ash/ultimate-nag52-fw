#include "can_egs51.h"

#include "driver/twai.h"
#include "driver/i2c.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"

#include "shifter/shifter_ewm.h"
#include "shifter/shifter_trrs.h"

Egs51Can::Egs51Can(const char* name, uint8_t tx_time_ms, uint32_t baud) : EgsBaseCan(name, tx_time_ms, baud) {
    ESP_LOGI("EGS51", "SETUP CALLED");

    switch (VEHICLE_CONFIG.shifter_style)
    {
    case (uint8_t)ShifterStyle::TRRS:
        shifter = new ShifterTrrs(&(this->can_init_status), this->name, &start_enable);
        break;
    default:
        shifter = new ShifterEwm(&(this->can_init_status), &ewm);
        break;
    }
    this->start_enable = true;
    this->gs218.TORQUE_REQ = 0xFE;
    this->gs218.bytes[7] = 0xFE;
    this->gs218.bytes[4] = 0x48;
    this->gs218.bytes[3] = 0x64;
}

WheelData Egs51Can::get_front_right_wheel(const uint32_t now, const uint32_t expire_time_ms) {  // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData Egs51Can::get_front_left_wheel(const uint32_t now, const uint32_t expire_time_ms) { // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData Egs51Can::get_rear_right_wheel(const uint32_t now, const uint32_t expire_time_ms) {
    BS_208_EGS51 bs208;
    if (this->esp51.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvailable;
        switch(bs208.DRTGHR) {
            case BS_208h_DRTGHR_EGS51::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHR_EGS51::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHR_EGS51::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHR_EGS51::SNV:
            default:
                break;
        }

        return WheelData {
            .double_rpm = bs208.DHR,
            .current_dir = d
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

WheelData Egs51Can::get_rear_left_wheel(const uint32_t now, const uint32_t expire_time_ms) {
    BS_208_EGS51 bs208;
    if (this->esp51.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvailable;
        switch(bs208.DRTGHL) {
            case BS_208h_DRTGHL_EGS51::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHL_EGS51::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHL_EGS51::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHL_EGS51::SNV:
            default:
                break;
        }

        return WheelData {
            .double_rpm = bs208.DHL,
            .current_dir = d
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

ShifterPosition Egs51Can::get_shifter_position(const uint32_t now, const uint32_t expire_time_ms) {
    return shifter->get_shifter_position(now, expire_time_ms);
}

EngineType Egs51Can::get_engine_type(const uint32_t now, const uint32_t expire_time_ms) {
    return EngineType::Unknown;
}

bool Egs51Can::get_engine_is_limp(const uint32_t now, const uint32_t expire_time_ms) { // TODO
    return false;
}

bool Egs51Can::get_kickdown(const uint32_t now, const uint32_t expire_time_ms) { // TODO
    return false;
}

uint8_t Egs51Can::get_pedal_value(const uint32_t now, const uint32_t expire_time_ms) { // TODO
    MS_210_EGS51 ms210;
    if (this->ms51.get_MS_210(now, expire_time_ms, &ms210)) {
        return ms210.PW;
    } else {
        return 0xFF;
    }
}

int Egs51Can::get_static_engine_torque(const uint32_t now, const uint32_t expire_time_ms) {
    MS_310_EGS51 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return ms310.IND_TORQUE*3;
    } else {
        return INT_MAX;
    }
    return INT_MAX;
}

int Egs51Can::get_driver_engine_torque(const uint32_t now, const uint32_t expire_time_ms) {
    // Don't think EGS51 supports this so just run with static torque for now
    return this->get_static_engine_torque(now, expire_time_ms);
}

int Egs51Can::get_maximum_engine_torque(const uint32_t now, const uint32_t expire_time_ms) {
    MS_310_EGS51 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return ms310.MAX_TORQUE*3;
    } else {
        return INT_MAX;
    }
    return INT_MAX;
}

int Egs51Can::get_minimum_engine_torque(const uint32_t now, const uint32_t expire_time_ms) {
    MS_310_EGS51 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return (int)ms310.MIN_TORQUE*3;
    } else {
        return INT_MAX;
    }
    return INT_MAX;
}

PaddlePosition Egs51Can::get_paddle_position(const uint32_t now, const uint32_t expire_time_ms) {
    return PaddlePosition::SNV;
}

int16_t Egs51Can::get_engine_coolant_temp(const uint32_t now, const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    int16_t res = INT16_MAX;
    if (this->ms51.get_MS_608(now, expire_time_ms, &ms608)) {
        if (ms608.T_MOT != UINT8_MAX) {
            res = ms608.T_MOT - 40;
        }
    }
    return res;
}

int16_t Egs51Can::get_engine_oil_temp(const uint32_t now, const uint32_t expire_time_ms) {
    MS_308_EGS51 ms308;
    int16_t res = INT16_MAX;
    if (this->ms51.get_MS_308(now, expire_time_ms, &ms308)) {
        if (ms308.T_OEL != UINT8_MAX) {
            res = ms308.T_OEL - 40;
        }
    }
    return res;
}

int16_t Egs51Can::get_engine_iat_temp(const uint32_t now, const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    int16_t res = INT16_MAX;
    if (this->ms51.get_MS_608(now, expire_time_ms, &ms608)) {
        if (ms608.T_LUFT != UINT8_MAX) {
            res = ms608.T_LUFT - 40;
        }
    }
    return res;
}

uint16_t Egs51Can::get_engine_rpm(const uint32_t now, const uint32_t expire_time_ms) {
    MS_308_EGS51 ms308;
    if (this->ms51.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.NMOT;
    } else {
        return UINT16_MAX;
    }
}

bool Egs51Can::get_is_starting(const uint32_t now, const uint32_t expire_time_ms) { // TODO
    return false;
}

bool Egs51Can::get_is_brake_pressed(const uint32_t now, const uint32_t expire_time_ms) {
    return false;
}

bool Egs51Can::get_profile_btn_press(const uint32_t now, const uint32_t expire_time_ms) {
    return false;
}

ProfileSwitchPos Egs51Can::get_shifter_ws_mode(const uint32_t now, const uint32_t expire_time_ms) {
    return this->shifter->get_shifter_profile_switch_pos(now, expire_time_ms);
}

void Egs51Can::set_clutch_status(TccClutchStatus status) {
    
}

void Egs51Can::set_actual_gear(GearboxGear actual) {
    switch(actual) {
        case GearboxGear::First:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D1;
            break;
        case GearboxGear::Second:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D2;
            break;
        case GearboxGear::Third:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D5;
            break;
        case GearboxGear::Park:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_P;
            break;
        case GearboxGear::Neutral:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_N;
            break;
        case GearboxGear::Reverse_First:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_R2;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_SNV;
            break;
    }
}

void Egs51Can::set_target_gear(GearboxGear target) {
    switch(target) {
        case GearboxGear::First:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D1;
            break;
        case GearboxGear::Second:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D2;
            break;
        case GearboxGear::Third:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D5;
            break;
        case GearboxGear::Park:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_P;
            break;
        case GearboxGear::Neutral:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_N;
            break;
        case GearboxGear::Reverse_First:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_R2;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_SNV;
            break;
    }
}

void Egs51Can::set_safe_start(bool can_start) {
    this->start_enable = can_start;
}

void Egs51Can::set_gearbox_temperature(uint16_t temp) {
}

void Egs51Can::set_input_shaft_speed(uint16_t rpm) {
}

void Egs51Can::set_is_all_wheel_drive(bool is_4wd) {
}

void Egs51Can::set_wheel_torque(uint16_t t) {
}

void Egs51Can::set_shifter_position(ShifterPosition pos) {
}

void Egs51Can::set_gearbox_ok(bool is_ok) {
}

void Egs51Can::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    if (control_type == TorqueRequestControlType::None) {
        this->gs218.TORQUE_REQ_EN = false;
        this->gs218.TORQUE_REQ = 0xFE;
    } else {
        // Just enable the request
        this->gs218.TORQUE_REQ_EN = true;
        this->gs218.TORQUE_REQ = amount_nm/3;
    }
}

void Egs51Can::set_error_check_status(SystemStatusCheck ssc) {
}


void Egs51Can::set_turbine_torque_loss(uint16_t loss_nm) {
}

void Egs51Can::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
}

void Egs51Can::set_drive_profile(GearboxProfile p) {
}

void Egs51Can::set_display_msg(GearboxMessage msg) {
}

void Egs51Can::set_wheel_torque_multi_factor(float ratio) {
}

/**
 * Parity calculation for torque numbers on GS218 and GS418
 * 
 * Each torque request member is a struct of 16 bits comprised of the following fields:
 * 1. Toggle bit (1 bit)
 * 2. Max request - bool (1 bit)
 * 3. Min request - bool (1 bit)
 * 4. Required torque - 13 bits
 */
inline bool calc_torque_parity(uint16_t s) {
    uint16_t p = s;
    p ^= (p >> 1);
    p ^= (p >> 2);
    p ^= (p >> 4);
    p ^= (p >> 8);
    return (p & 1) == 1;
}

/**
 * @brief void tx_frames() override;
        void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) override;
        void on_rx_done(uint64_t now_ts) override;
 * 
 */

void Egs51Can::tx_frames() {
    tx.data_length_code = 8; // Always
    GS_218_EGS51 gs_218tx;
    // Copy current CAN frame values to here so we don't
    // accidentally modify parity calculations
    gs_218tx = {gs218.raw};

    // Firstly we have to deal with toggled bits!
    // As toggle bits need to be toggled every 40ms,
    // and egs52 Tx interval is 20ms,
    // we can achieve this with 2 booleans
    //gs_218tx.set_MTGL_EGS(toggle);
    // Now do parity calculations
    //gs_218tx.set_MPAR_EGS(calc_torque_parity(gs_218tx.raw >> 48));
    if (time_to_toggle) {
        toggle = !toggle;
    }
    time_to_toggle = !time_to_toggle;
    
    // Now set CVN Counter (Increases every frame)
    gs_218tx.FEHLER = cvn_counter;
    cvn_counter++;
    tx.identifier = GS_218_EGS51_CAN_ID;
    tx.data_length_code = 6;
    to_bytes(gs_218tx.raw, tx.data);
    twai_transmit(&tx, 5);
}

void Egs51Can::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) {
    if (this->ms51.import_frames(data, id, timestamp)) {
    } else if (this->esp51.import_frames(data, id, timestamp)) {
    } else if (this->ewm.import_frames(data, id, timestamp)) {
    }
}

void Egs51Can::on_rx_done(const uint32_t now_ts) {
    if(ShifterStyle::TRRS == (ShifterStyle)VEHICLE_CONFIG.shifter_style) {
        (static_cast<ShifterTrrs*>(shifter))->update_shifter_position(now_ts);
    }
}