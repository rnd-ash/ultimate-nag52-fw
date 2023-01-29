#include "can_egs51.h"

#include "driver/twai.h"
#include "gearbox_config.h"
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
    this->gs218.set_TORQUE_REQ(0xFE);
    this->gs218.bytes[7] = 0xFE;
    this->gs218.bytes[4] = 0x48;
    this->gs218.bytes[3] = 0x64;
}

WheelData Egs51Can::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData Egs51Can::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData Egs51Can::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_208EGS51 bs208;
    if (this->esp51.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvailable;
        switch(bs208.get_DRTGHR()) {
            case BS_208h_DRTGHREGS51::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHREGS51::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHREGS51::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHREGS51::SNV:
            default:
                break;
        }

        return WheelData {
            .double_rpm = bs208.get_DHR(),
            .current_dir = d
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

WheelData Egs51Can::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_208EGS51 bs208;
    if (this->esp51.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvailable;
        switch(bs208.get_DRTGHL()) {
            case BS_208h_DRTGHLEGS51::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHLEGS51::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHLEGS51::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHLEGS51::SNV:
            default:
                break;
        }

        return WheelData {
            .double_rpm = bs208.get_DHL(),
            .current_dir = d
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

ShifterPosition Egs51Can::get_shifter_position(uint64_t now, uint64_t expire_time_ms) {
    return shifter->get_shifter_position(now, expire_time_ms);
}

EngineType Egs51Can::get_engine_type(uint64_t now, uint64_t expire_time_ms) {
    return EngineType::Unknown;
}

bool Egs51Can::get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs51Can::get_kickdown(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

uint8_t Egs51Can::get_pedal_value(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_210EGS51 ms210;
    if (this->ms51.get_MS_210(now, expire_time_ms, &ms210)) {
        return ms210.get_PW();
    } else {
        return 0xFF;
    }
}

int Egs51Can::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_310EGS51 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return (int)ms310.get_STA_TORQUE()*2;
    } else {
        return INT_MAX;
    }
    return INT_MAX;
}

int Egs51Can::get_driver_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    return this->get_static_engine_torque(now, expire_time_ms);
}

int Egs51Can::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_310EGS51 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return (int)ms310.get_MAX_TORQUE()*2;
    } else {
        return INT_MAX;
    }
    return INT_MAX;
}

int Egs51Can::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    return 0; // Always 0 on W210 as ECUs do NOT calculate inertia
}

PaddlePosition Egs51Can::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    return PaddlePosition::SNV;
}

int16_t Egs51Can::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    MS_608EGS51 ms608;
    if (this->ms51.get_MS_608(now, expire_time_ms, &ms608)) {
        return ms608.get_T_MOT() - 40;
    } else {
        return UINT16_MAX;
    }
}

int16_t Egs51Can::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_308EGS51 ms308;
    if (this->ms51.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.get_T_OEL() - 40;
    } else {
        return UINT16_MAX;
    }
}

uint16_t Egs51Can::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    MS_308EGS51 ms308;
    if (this->ms51.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.get_NMOT();
    } else {
        return UINT16_MAX;
    }
}

bool Egs51Can::get_is_starting(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs51Can::get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

bool Egs51Can::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

void Egs51Can::set_clutch_status(ClutchStatus status) {
    
}

void Egs51Can::set_actual_gear(GearboxGear actual) {
    switch(actual) {
        case GearboxGear::First:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D1);
            break;
        case GearboxGear::Second:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D2);
            break;
        case GearboxGear::Third:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D3);
            break;
        case GearboxGear::Fourth:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D4);
            break;
        case GearboxGear::Fifth:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D5);
            break;
        case GearboxGear::Park:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_P);
            break;
        case GearboxGear::Neutral:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_N);
            break;
        case GearboxGear::Reverse_First:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_R);
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_R2);
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_SNV);
            break;
    }
}

void Egs51Can::set_target_gear(GearboxGear target) {
    switch(target) {
        case GearboxGear::First:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D1);
            break;
        case GearboxGear::Second:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D2);
            break;
        case GearboxGear::Third:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D3);
            break;
        case GearboxGear::Fourth:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D4);
            break;
        case GearboxGear::Fifth:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D5);
            break;
        case GearboxGear::Park:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_P);
            break;
        case GearboxGear::Neutral:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_N);
            break;
        case GearboxGear::Reverse_First:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_R);
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_R2);
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_SNV);
            break;
    }
}

void Egs51Can::set_safe_start(bool can_start) {
    this->start_enable = can_start;
}

void Egs51Can::set_race_start(bool race_start) {
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

void Egs51Can::set_torque_request(TorqueRequest request) {
    if (request == TorqueRequest::None) {
        this->gs218.set_TORQUE_REQ_EN(false);
    } else {
        // Just enable the request
        this->gs218.set_TORQUE_REQ_EN(true);
    }
}

void Egs51Can::set_requested_torque(uint16_t torque_nm) {
    if (torque_nm == 0 && gs218.get_TORQUE_REQ_EN() == false) {
        this->gs218.set_TORQUE_REQ(0xFE);
    } else {
        this->gs218.set_TORQUE_REQ(torque_nm/2);
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
    twai_message_t tx;
    tx.data_length_code = 8; // Always
    GS_218EGS51 gs_218tx;
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
    gs_218tx.set_FEHLER(cvn_counter);
    cvn_counter++;
    tx.identifier = GS_218EGS51_CAN_ID;
    tx.data_length_code = 6;
    to_bytes(gs_218tx.raw, tx.data);
    twai_transmit(&tx, 5);
}

void Egs51Can::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) {
    if (this->ms51.import_frames(data, id, timestamp)) {
    } else if (this->esp51.import_frames(data, id, timestamp)) {
    } else if (this->ewm.import_frames(data, id, timestamp)) {
    }
}

void Egs51Can::on_rx_done(uint64_t now_ts) {
    if(ShifterStyle::TRRS == VEHICLE_CONFIG.shifter_style) {
        (static_cast<ShifterTrrs*>(shifter))->update_shifter_position(now_ts);
    }
}