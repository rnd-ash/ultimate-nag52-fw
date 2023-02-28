#include "can_egs52.h"
#include "driver/twai.h"
#include "gearbox_config.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"

Egs52Can::Egs52Can(const char* name, uint8_t tx_time_ms, uint32_t baud) : EgsBaseCan(name, tx_time_ms, baud) {
    // Set default values
    this->set_target_gear(GearboxGear::SignalNotAvailable);
    this->set_actual_gear(GearboxGear::SignalNotAvailable);
    this->set_shifter_position(ShifterPosition::SignalNotAvailable);
    this->gs218.set_GIC(GS_218h_GIC::G_SNV);
    gs218.set_CALID_CVN_AKT(true);
    gs218.set_G_G(true);
    // Set profile to N/A for now
    this->set_drive_profile(GearboxProfile::Underscore);
    // Set no message
    this->set_display_msg(GearboxMessage::None);
    this->gs218.set_SCHALT(true);
    this->gs218.set_MKRIECH(0xFF);

    if (VEHICLE_CONFIG.is_four_matic != 0) {
        this->gs418.set_ALLRAD(true);
    } else {
        this->gs418.set_ALLRAD(false);
    }
    this->gs418.set_FRONT(false); // Primary rear wheel drive
    this->gs418.set_CVT(false); // Not CVT gearbox]
    if (VEHICLE_CONFIG.is_large_nag != 0) {
        this->gs418.set_MECH(GS_418h_MECH::GROSS);
    } else {
        this->gs418.set_MECH(GS_418h_MECH::KLEIN);
    }
    this->gs218.set_ALF(true); // Fix for KG systems where cranking would stop when TCU turns on


    // Covers setting NAB, a couple unknown but static values,
    // and Input RPM to 0 
    gs338.raw = 0xFFFF1FFF00FF0000u;
}

WheelData Egs52Can::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData Egs52Can::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData Egs52Can::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_208 bs208;
    if (this->esp_ecu.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvailable;
        switch(bs208.get_DRTGHR()) {
            case BS_208h_DRTGHR::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHR::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHR::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHR::SNV:
            default:
                break;
        }

        // Fix for some cars where SNV even with valid wheel speed
        if (bs208.get_DHR() != 0 && d == WheelDirection::SignalNotAvailable) {
            d = WheelDirection::Forward;
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

WheelData Egs52Can::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_208 bs208;
    if (this->esp_ecu.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvailable;
        switch(bs208.get_DRTGHL()) {
            case BS_208h_DRTGHL::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHL::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHL::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHL::SNV:
            default:
                break;
        }

        // Fix for some cars where SNV even with valid wheel speed
        if (bs208.get_DHL() != 0 && d == WheelDirection::SignalNotAvailable) {
            d = WheelDirection::Forward;
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

ShifterPosition Egs52Can::get_shifter_position_ewm(uint64_t now, uint64_t expire_time_ms) {
    EWM_230 dest;
    if (this->ewm_ecu.get_EWM_230(now, expire_time_ms, &dest)) {
        switch (dest.get_WHC()) {
            case EWM_230h_WHC::D:
                return ShifterPosition::D;
            case EWM_230h_WHC::N:
                return ShifterPosition::N;
            case EWM_230h_WHC::R:
                return ShifterPosition::R;
            case EWM_230h_WHC::P:
                return ShifterPosition::P;
            case EWM_230h_WHC::PLUS:
                return ShifterPosition::PLUS;
            case EWM_230h_WHC::MINUS:
                return ShifterPosition::MINUS;
            case EWM_230h_WHC::N_ZW_D:
                return ShifterPosition::N_D;
            case EWM_230h_WHC::R_ZW_N:
                return ShifterPosition::R_N;
            case EWM_230h_WHC::P_ZW_R:
                return ShifterPosition::P_R;
            case EWM_230h_WHC::SNV:
            default:
                return ShifterPosition::SignalNotAvailable;
        }
    } else {
        return ShifterPosition::SignalNotAvailable;
    }
}

EngineType Egs52Can::get_engine_type(uint64_t now, uint64_t expire_time_ms) {
    MS_608 ms608;
    // This signal can be valid for up to 1000ms
    if (this->ecu_ms.get_MS_608(now, expire_time_ms, &ms608)) {
        switch (ms608.get_FCOD_MOT()) {
            case MS_608h_FCOD_MOT::OM611DE22LA100:
            case MS_608h_FCOD_MOT::OM640DE20LA60:
            case MS_608h_FCOD_MOT::OM642DE30LA160:
                return EngineType::Diesel;
            default:
                return EngineType::Unknown;
        }
    } else {
        return EngineType::Unknown;
    }
}

bool Egs52Can::get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs52Can::get_kickdown(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

uint8_t Egs52Can::get_pedal_value(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_210 ms210;
    // This signal can be valid for up to 1000ms
    if (this->ecu_ms.get_MS_210(now, expire_time_ms, &ms210)) {
        return ms210.get_PW();
    } else {
        return 0xFF;
    }
}

int Egs52Can::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_312 ms312;
    if (this->ecu_ms.get_MS_312(now, expire_time_ms, &ms312)) {
        return ((int)ms312.get_M_STA() / 4) - 500;
    }
    return INT_MAX;
}

int Egs52Can::get_driver_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    MS_212 ms212;
    if (this->ecu_ms.get_MS_212(now, expire_time_ms, &ms212)) {
        return ((int)ms212.get_M_FV() / 4) - 500;
    }
    return INT_MAX;
}

int Egs52Can::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_312 ms312;
    if (this->ecu_ms.get_MS_312(now, expire_time_ms, &ms312)) {
        return ((int)ms312.get_M_MAX() / 4) - 500;
    }
    return INT_MAX;
}

int Egs52Can::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_312 ms312;
    if (this->ecu_ms.get_MS_312(now, expire_time_ms, &ms312)) {
        return ((int)ms312.get_M_MIN() / 4) - 500;
    }
    return INT_MAX;
}

uint8_t Egs52Can::get_ac_torque_loss(uint64_t now, uint64_t expire_time_ms) {
    KLA_410 kl410;
    if (this->ezs_ecu.get_KLA_410(now, expire_time_ms, &kl410)) {
        uint8_t ret = kl410.get_M_KOMP();
        if (ret != UINT8_MAX) {
            ret /= 4;
        }
        return ret;
    }
    return UINT8_MAX;
}

PaddlePosition Egs52Can::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    SBW_232 sbw;
    if (misc_ecu.get_SBW_232(now, expire_time_ms, &sbw)) { // 50ms timeout
        switch (sbw.get_LRT_PM3()) {
            case SBW_232h_LRT_PM3::PLUS:
                return PaddlePosition::Plus;
            case SBW_232h_LRT_PM3::MINUS:
                return PaddlePosition::Minus;
            case SBW_232h_LRT_PM3::SNV:
                return PaddlePosition::SNV;
            default:
                return PaddlePosition::None;
        }
    } else {
        return PaddlePosition::SNV;
    }
}

int16_t Egs52Can::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    MS_608 ms608;
    if (ecu_ms.get_MS_608(now, expire_time_ms, &ms608)) {
        return ms608.get_T_MOT()-40;
    } else {
        return INT16_MAX;
    }
}

int16_t Egs52Can::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_308 ms308;
    if (ecu_ms.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.get_T_OEL()-40;
    } else {
        return INT16_MAX;
    }
}

uint16_t Egs52Can::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    MS_308 ms308;
    if (ecu_ms.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.get_NMOT();
    } else {
        return UINT16_MAX;
    }
}

bool Egs52Can::get_is_starting(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_308 ms308;
    if (ecu_ms.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.get_ANL_LFT();
    } else {
        return false;
    }
}

bool Egs52Can::get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) {
    BS_200 bs200;
    if (this->esp_ecu.get_BS_200(now, expire_time_ms, &bs200)) {
        return bs200.get_BLS() == BS_200h_BLS::BREMSE_BET;
    } else {
        return false;
    }
}

bool state = false;
bool Egs52Can::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    EWM_230 ewm230;
    if (this->ewm_ecu.get_EWM_230(now, expire_time_ms, &ewm230)) {
        if (ewm230.get_FPT()) {
            if (!state) {
                this->esp_toggle = !this->esp_toggle;
            }
            state = true;
        } else {
            state = false;
        }
        return ewm230.get_FPT();
    } else {
        return false;
    }
}

bool Egs52Can::get_shifter_ws_mode(uint64_t now, uint64_t expire_time_ms) {
    EWM_230 ewm230;
    if (this->ewm_ecu.get_EWM_230(now, expire_time_ms, &ewm230)) {
        return ewm230.get_W_S();
    } else {
        return false;
    }
}

// TerminalStatus Egs52Can::get_terminal_15(uint64_t now, uint64_t expire_time_ms) {
//     EZS_240 ezs240;
//     if (this->ezs_ecu.get_EZS_240(now, expire_time_ms, &ezs240)) {
//         return ezs240.get_KL_15() ? TerminalStatus::On : TerminalStatus::Off;
//     } else {
//         return TerminalStatus::SNA;
//     }
// }

uint16_t Egs52Can::get_fuel_flow_rate(uint64_t now, uint64_t expire_time_ms) {
    MS_608 ms608;
    if (this->ecu_ms.get_MS_608(now, expire_time_ms, &ms608)) {
        return (uint16_t)((float)ms608.get_VB()*0.85);
    } else {
        return 0;
    }
}

TransferCaseState Egs52Can::get_transfer_case_state(uint64_t now, uint64_t expire_time_ms) {
    VG_428 vg428;
    if (this->misc_ecu.get_VG_428(now, expire_time_ms, &vg428)) {
        switch (vg428.get_VG_GANG()) {
            case VG_428h_VG_GANG::HI:
                return TransferCaseState::Hi;
            case VG_428h_VG_GANG::LO:
                return TransferCaseState::Low;
            case VG_428h_VG_GANG::N:
                return TransferCaseState::Neither;
            case VG_428h_VG_GANG::SH_IPG:
                return TransferCaseState::Switching;
            case VG_428h_VG_GANG::SNV:
            default:
                return TransferCaseState::SNA;
        }
    } else {
        return TransferCaseState::SNA;
    }
}

void Egs52Can::set_clutch_status(ClutchStatus status) {
    switch(status) {
        case ClutchStatus::Open:
            gs218.set_K_G_B(false);
            gs218.set_K_O_B(true);
            gs218.set_K_S_B(false);
            break;
        case ClutchStatus::Slipping:
            gs218.set_K_G_B(false);
            gs218.set_K_O_B(false);
            gs218.set_K_S_B(true);
            break;
        case ClutchStatus::Closed:
            gs218.set_K_G_B(true);
            gs218.set_K_O_B(false);
            gs218.set_K_S_B(false);
            break;
        default:
            break;
    }
}

void Egs52Can::set_actual_gear(GearboxGear actual) {
    switch (actual) {
        case GearboxGear::Park:
            this->gs418.set_GIC(GS_418h_GIC::G_P);
            this->gs218.set_GIC(GS_218h_GIC::G_P);
            break;
        case GearboxGear::Reverse_First:
            this->gs418.set_GIC(GS_418h_GIC::G_R);
            this->gs218.set_GIC(GS_218h_GIC::G_R);
            break;
        case GearboxGear::Reverse_Second:
            this->gs418.set_GIC(GS_418h_GIC::G_R2);
            this->gs218.set_GIC(GS_218h_GIC::G_R2);
            break;
        case GearboxGear::Neutral:
            this->gs418.set_GIC(GS_418h_GIC::G_N);
            this->gs218.set_GIC(GS_218h_GIC::G_N);
            break;
        case GearboxGear::First:
            this->gs418.set_GIC(GS_418h_GIC::G_D1);
            this->gs218.set_GIC(GS_218h_GIC::G_D1);
            break;
        case GearboxGear::Second:
            this->gs418.set_GIC(GS_418h_GIC::G_D2);
            this->gs218.set_GIC(GS_218h_GIC::G_D2);
            break;
        case GearboxGear::Third:
            this->gs418.set_GIC(GS_418h_GIC::G_D3);
            this->gs218.set_GIC(GS_218h_GIC::G_D3);
            break;
        case GearboxGear::Fourth:
            this->gs418.set_GIC(GS_418h_GIC::G_D4);
            this->gs218.set_GIC(GS_218h_GIC::G_D4);
            break;
        case GearboxGear::Fifth:
            this->gs418.set_GIC(GS_418h_GIC::G_D5);
            this->gs218.set_GIC(GS_218h_GIC::G_D5);
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs418.set_GIC(GS_418h_GIC::G_SNV);
            this->gs218.set_GIC(GS_218h_GIC::G_SNV);
            break;
    }
}

void Egs52Can::set_target_gear(GearboxGear target) {
    switch (target) {
        case GearboxGear::Park:
            this->gs418.set_GZC(GS_418h_GZC::G_P);
            this->gs218.set_GZC(GS_218h_GZC::G_P);
            break;
        case GearboxGear::Reverse_First:
            this->gs418.set_GZC(GS_418h_GZC::G_R);
            this->gs218.set_GZC(GS_218h_GZC::G_R);
            break;
        case GearboxGear::Reverse_Second:
            this->gs418.set_GZC(GS_418h_GZC::G_R2);
            this->gs218.set_GZC(GS_218h_GZC::G_R2);
            break;
        case GearboxGear::Neutral:
            this->gs418.set_GZC(GS_418h_GZC::G_N);
            this->gs218.set_GZC(GS_218h_GZC::G_N);
            break;
        case GearboxGear::First:
            this->gs418.set_GZC(GS_418h_GZC::G_D1);
            this->gs218.set_GZC(GS_218h_GZC::G_D1);
            break;
        case GearboxGear::Second:
            this->gs418.set_GZC(GS_418h_GZC::G_D2);
            this->gs218.set_GZC(GS_218h_GZC::G_D2);
            break;
        case GearboxGear::Third:
            this->gs418.set_GZC(GS_418h_GZC::G_D3);
            this->gs218.set_GZC(GS_218h_GZC::G_D3);
            break;
        case GearboxGear::Fourth:
            this->gs418.set_GZC(GS_418h_GZC::G_D4);
            this->gs218.set_GZC(GS_218h_GZC::G_D4);
            break;
        case GearboxGear::Fifth:
            this->gs418.set_GZC(GS_418h_GZC::G_D5);
            this->gs218.set_GZC(GS_218h_GZC::G_D5);
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs418.set_GZC(GS_418h_GZC::G_SNV);
            this->gs218.set_GZC(GS_218h_GZC::G_SNV);
            break;
    }
}

void Egs52Can::set_safe_start(bool can_start) {
    this->gs218.set_ALF(can_start);   
}

void Egs52Can::set_race_start(bool race_start) {
    this->gs218.set_KS(race_start);
}

void Egs52Can::set_gearbox_temperature(uint16_t temp) {
    this->gs418.set_T_GET((temp+50) & 0xFF);
}

void Egs52Can::set_input_shaft_speed(uint16_t rpm) {
    gs338.set_NTURBINE(rpm);
}

void Egs52Can::set_is_all_wheel_drive(bool is_4wd) {
    this->gs418.set_ALLRAD(is_4wd);
}

void Egs52Can::set_wheel_torque(uint16_t t) {

}

void Egs52Can::set_shifter_position(ShifterPosition pos) {
    switch (pos) {
        case ShifterPosition::P:
            gs418.set_WHST(GS_418h_WHST::P);
            break;
        case ShifterPosition::R:
            gs418.set_WHST(GS_418h_WHST::R);
            break;
        case ShifterPosition::N:
            gs418.set_WHST(GS_418h_WHST::N);
            break;
        case ShifterPosition::D:
            gs418.set_WHST(GS_418h_WHST::D);
            break;
        case ShifterPosition::SignalNotAvailable:
            gs418.set_WHST(GS_418h_WHST::SNV);
            break;
        default: 
            break;
    }
}

void Egs52Can::set_gearbox_ok(bool is_ok) {
    gs218.set_GET_OK(is_ok); // Gearbox OK
    gs218.set_GSP_OK(is_ok); // Gearbox profile OK
    gs218.set_GS_NOTL(!is_ok); // Emergency mode activated
}

void Egs52Can::set_torque_request(TorqueRequest request, float amount_nm) {
    bool dyn0 = request != TorqueRequest::None; // Markes torque request active
    bool dyn1 = false;
    bool min = false;
    bool max = false;
    uint16_t trq = 0;
    if (request != TorqueRequest::None) {
        trq = (amount_nm + 500) * 4;
    } else {
        trq = 0;
    }
    // Type of request bit
    switch(request) {
        case TorqueRequest::Exact:
        case TorqueRequest::ExactFast:
            min = true;
            max = true;
            break;
        case TorqueRequest::LessThan:
        case TorqueRequest::LessThanFast:
            min = true;
            max = false;
            break;
        case TorqueRequest::MoreThan:
        case TorqueRequest::MoreThanFast:
            min = false;
            max = true;
            break;
        case TorqueRequest::None:
        default:
            min = false;
            max = false;
            break;
    }
    // Request engage type bit
    switch(request) {
        case TorqueRequest::Exact:
        case TorqueRequest::LessThan:
        case TorqueRequest::MoreThan:
            dyn1 = false;
            break;
        case TorqueRequest::ExactFast:
        case TorqueRequest::LessThanFast:
        case TorqueRequest::MoreThanFast:
            dyn1 = true;
            break;
        case TorqueRequest::None:
        default:
            dyn1 = false;
            break;
    }
    gs218.set_M_EGS(trq);
    gs218.set_DYN0_AMR_EGS(dyn0);
    gs218.set_DYN1_EGS(dyn1);
    gs218.set_MMAX_EGS(max);
    gs218.set_MMIN_EGS(min);
}

void Egs52Can::set_error_check_status(SystemStatusCheck ssc) {
    switch(ssc) {
        case SystemStatusCheck::Error:
            gs218.set_FEHLPRF_ST(GS_218h_FEHLPRF_ST::ERROR);
            break;
        case SystemStatusCheck::Waiting:
            gs218.set_FEHLPRF_ST(GS_218h_FEHLPRF_ST::WAIT);
            break;
        case SystemStatusCheck::OK:
            gs218.set_FEHLPRF_ST(GS_218h_FEHLPRF_ST::OK);
            break;
        default:
            break;
    }
}


void Egs52Can::set_turbine_torque_loss(uint16_t loss_nm) {
    gs418.set_M_VERL(loss_nm*4);
}

void Egs52Can::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
    switch(g) {
        case GearboxDisplayGear::P:
            this->gs418.set_FSC('P');
            break;
        case GearboxDisplayGear::N:
            this->gs418.set_FSC('N');
            break;
        case GearboxDisplayGear::R:
            this->gs418.set_FSC('R');
            break;
        case GearboxDisplayGear::One:
            this->gs418.set_FSC('1');
            break;
        case GearboxDisplayGear::Two:
            this->gs418.set_FSC('2');
            break;
        case GearboxDisplayGear::Three:
            this->gs418.set_FSC('3');
            break;
        case GearboxDisplayGear::Four:
            this->gs418.set_FSC('4');
            break;
        case GearboxDisplayGear::Five:
            this->gs418.set_FSC('5');
            break;
        case GearboxDisplayGear::A:
            this->gs418.set_FSC('A');
            break;
        case GearboxDisplayGear::D:
            this->gs418.set_FSC('D');
            break;
        case GearboxDisplayGear::Failure:
            this->gs418.set_FSC('F');
            break;
        case GearboxDisplayGear::SNA:
        default:
            this->gs418.set_FSC(' ');
            break;

    }
}

void Egs52Can::set_drive_profile(GearboxProfile p) {
    this->curr_profile_bit = p;
    switch (p) {
        case GearboxProfile::Agility:
            gs418.set_FPC('A');
            break;
        case GearboxProfile::Comfort:
            gs418.set_FPC('C');
            break;
        case GearboxProfile::Winter:
            gs418.set_FPC('W');
            break;
        case GearboxProfile::Failure:
            gs418.set_FPC('F');
            break;
        case GearboxProfile::Standard:
            gs418.set_FPC('S');
            break;
        case GearboxProfile::Manual:
            gs418.set_FPC('M');
            break;
        case GearboxProfile::Individual:
            gs418.set_FPC('I');
            break;
        case GearboxProfile::Race:
            gs418.set_FPC('R');
            break;
        case GearboxProfile::Underscore:
            gs418.set_FPC('_');
            break;
        default:
            break;
    }
    // Update display message as well
    this->set_display_msg(this->curr_message);
}

void Egs52Can::set_solenoid_pwm(uint16_t duty, SolenoidName s) {
    switch (s) {
        case SolenoidName::Y3:
            gs558.set_Y3_DUTY(duty >> 8);
            break;
        case SolenoidName::Y4:
            gs558.set_Y4_DUTY(duty >> 8);
            break;
        case SolenoidName::Y5:
            gs558.set_Y5_DUTY(duty >> 8);
            break;
        case SolenoidName::SPC:
            gs558.set_SPC_CURRENT(duty);
            break;
        case SolenoidName::MPC:
            gs558.set_MPC_CURRENT(duty);
            break;
        case SolenoidName::TCC:
            gs558.set_TCC_DUTY(duty >> 4);
            break;
        default:
            break;
    }
}

void Egs52Can::set_spc_pressure(uint16_t p) {
    this->gs668.set_SPC_PRESSURE_EST(p);
}

void Egs52Can::set_mpc_pressure(uint16_t p) {
    this->gs668.set_MPC_PRESSURE_EST(p);
}

void Egs52Can::set_tcc_pressure(uint16_t p) {
    this->gs668.set_TCC_PRESSURE_EST(p);
}

void Egs52Can::set_shift_stage(uint8_t stage, bool is_ramp) {
    this->gs668.set_SHIFT_STAGE(stage);
}

void Egs52Can::set_gear_disagree(uint8_t count) {
    this->gs668.set_GEAR_DISAGREE(count);
}

void Egs52Can::set_gear_ratio(int16_t g100) {
    this->gs558.set_RATIO(g100);
}

void Egs52Can::set_abort_shift(bool is_aborting){
    this->gs218.set_GZC(GS_218h_GZC::G_ABBRUCH);
}

void Egs52Can::set_display_msg(GearboxMessage msg) {
    /*
    this->curr_message = msg;
    if (this->curr_profile_bit == GearboxProfile::Agility) {
        switch (msg) {
            case GearboxMessage::None:
                gs418.set_FPC(GS_418h_FPC::A);
                break;
            case GearboxMessage::ActuateParkingBreak:
                gs418.set_FPC(GS_418h_FPC::A_MGFB_WT);
                break;
            case GearboxMessage::ShiftLeverToN:
                gs418.set_FPC(GS_418h_FPC::A_MGN);
                break;
            case GearboxMessage::ActivateGear:
                gs418.set_FPC(GS_418h_FPC::A_MGBB);
                break;
            case GearboxMessage::RequestGearAgain:
                gs418.set_FPC(GS_418h_FPC::A_MGGEA);
                break;
            case GearboxMessage::InsertToNToStart:
                gs418.set_FPC(GS_418h_FPC::A_MGZSN);
                break;
            case GearboxMessage::Upshift:
                gs418.set_FPC(GS_418h_FPC::HOCH);
                break;
            case GearboxMessage::Downshift:
                gs418.set_FPC(GS_418h_FPC::RUNTER);
                break;
            case GearboxMessage::VisitWorkshop:
                gs418.set_FPC(GS_418h_FPC::A_MGW);
                break;
            default:
                break;
        }
    } else if (this->curr_profile_bit ==  GearboxProfile::Comfort) {
        switch (msg) {
            case GearboxMessage::None:
                gs418.set_FPC(GS_418h_FPC::C);
                break;
            case GearboxMessage::ActuateParkingBreak:
                gs418.set_FPC(GS_418h_FPC::C_MGFB_WT);
                break;
            case GearboxMessage::ShiftLeverToN:
                gs418.set_FPC(GS_418h_FPC::C_MGN);
                break;
            case GearboxMessage::ActivateGear:
                gs418.set_FPC(GS_418h_FPC::C_MGBB);
                break;
            case GearboxMessage::RequestGearAgain:
                gs418.set_FPC(GS_418h_FPC::C_MGGEA);
                break;
            case GearboxMessage::InsertToNToStart:
                gs418.set_FPC(GS_418h_FPC::C_MGZSN);
                break;
            case GearboxMessage::Upshift:
                gs418.set_FPC(GS_418h_FPC::HOCH);
                break;
            case GearboxMessage::Downshift:
                gs418.set_FPC(GS_418h_FPC::RUNTER);
                break;
            case GearboxMessage::VisitWorkshop:
                gs418.set_FPC(GS_418h_FPC::C_MGW);
                break;
            default:
                break;
        }
    } else if (this->curr_profile_bit ==  GearboxProfile::Winter) {
         switch (msg) {
            case GearboxMessage::None:
                gs418.set_FPC(GS_418h_FPC::W);
                break;
            case GearboxMessage::ShiftLeverToN:
                gs418.set_FPC(GS_418h_FPC::W_MGN);
                break;
            case GearboxMessage::Upshift:
                gs418.set_FPC(GS_418h_FPC::HOCH);
                break;
            case GearboxMessage::Downshift:
                gs418.set_FPC(GS_418h_FPC::RUNTER);
                break;
            case GearboxMessage::VisitWorkshop:
                gs418.set_FPC(GS_418h_FPC::W_MGW);
                break;
            case GearboxMessage::ActuateParkingBreak: // Unsupported in 'W'
            case GearboxMessage::ActivateGear: // Unsupported in 'W'
            case GearboxMessage::RequestGearAgain: // Unsupported in 'W'
            case GearboxMessage::InsertToNToStart:// Unsupported in 'W'
            default:
                break;
        }
    } else if (this->curr_profile_bit ==  GearboxProfile::Failure) {
         switch (msg) {
            case GearboxMessage::None:
                gs418.set_FPC(GS_418h_FPC::F);
                break;
            case GearboxMessage::Upshift:
                gs418.set_FPC(GS_418h_FPC::HOCH);
                break;
            case GearboxMessage::Downshift:
                gs418.set_FPC(GS_418h_FPC::RUNTER);
                break;
            case GearboxMessage::VisitWorkshop:
                gs418.set_FPC(GS_418h_FPC::F_MGW);
                break;
            case GearboxMessage::ActuateParkingBreak: // Unsupported in 'F'
            case GearboxMessage::ShiftLeverToN: // Unsupported in 'F'
            case GearboxMessage::ActivateGear: // Unsupported in 'F'
            case GearboxMessage::RequestGearAgain: // Unsupported in 'F'
            case GearboxMessage::InsertToNToStart: // Unsupported in 'F'
            default:
                break;
        }
    } else if (this->curr_profile_bit ==  GearboxProfile::Standard) {
        switch (msg) {
            case GearboxMessage::None:
                gs418.set_FPC(GS_418h_FPC::S);
                break;
            case GearboxMessage::ActuateParkingBreak:
                gs418.set_FPC(GS_418h_FPC::S_MGFB_WT);
                break;
            case GearboxMessage::ShiftLeverToN:
                gs418.set_FPC(GS_418h_FPC::S_MGN);
                break;
            case GearboxMessage::ActivateGear:
                gs418.set_FPC(GS_418h_FPC::S_MGBB);
                break;
            case GearboxMessage::RequestGearAgain:
                gs418.set_FPC(GS_418h_FPC::S_MGGEA);
                break;
            case GearboxMessage::InsertToNToStart:
                gs418.set_FPC(GS_418h_FPC::S_MGZSN);
                break;
            case GearboxMessage::Upshift:
                gs418.set_FPC(GS_418h_FPC::HOCH);
                break;
            case GearboxMessage::Downshift:
                gs418.set_FPC(GS_418h_FPC::RUNTER);
                break;
            case GearboxMessage::VisitWorkshop:
                gs418.set_FPC(GS_418h_FPC::S_MGW);
                break;
            default:
                break;
        }
    } else if (this->curr_profile_bit ==  GearboxProfile::Manual) {
        switch (msg) {
            case GearboxMessage::None:
                gs418.set_FPC(GS_418h_FPC::M);
                break;
            case GearboxMessage::ShiftLeverToN:
                gs418.set_FPC(GS_418h_FPC::M_MGN);
                break;
            case GearboxMessage::Upshift:
                gs418.set_FPC(GS_418h_FPC::HOCH);
                break;
            case GearboxMessage::Downshift:
                gs418.set_FPC(GS_418h_FPC::RUNTER);
                break;
            case GearboxMessage::VisitWorkshop:
                gs418.set_FPC(GS_418h_FPC::M_MGW);
                break;
            case GearboxMessage::ActuateParkingBreak: // Unsupported in 'M'
            case GearboxMessage::ActivateGear: // Unsupported in 'M'
            case GearboxMessage::RequestGearAgain: // Unsupported in 'M'
            case GearboxMessage::InsertToNToStart: // Unsupported in 'M'
            default:
                break;
        }
    } else if (this->curr_profile_bit ==  GearboxProfile::Underscore) {
        switch (msg) {
            case GearboxMessage::ShiftLeverToN:
                gs418.set_FPC(GS_418h_FPC::__MGN);
                break;
            case GearboxMessage::Upshift:
                gs418.set_FPC(GS_418h_FPC::HOCH);
                break;
            case GearboxMessage::Downshift:
                gs418.set_FPC(GS_418h_FPC::RUNTER);
                break;
            case GearboxMessage::VisitWorkshop:
                gs418.set_FPC(GS_418h_FPC::__MGW);
                break;
            case GearboxMessage::ActuateParkingBreak: // Unsupported in '_'
            case GearboxMessage::ActivateGear: // Unsupported in '_'
            case GearboxMessage::RequestGearAgain: // Unsupported in '_'
            case GearboxMessage::InsertToNToStart: // Unsupported in '_'
            default:
                break;
        }
    }
    */
}

void Egs52Can::set_wheel_torque_multi_factor(float ratio) {
    gs418.set_FMRAD(ratio * 100);
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

void Egs52Can::tx_frames() {
    twai_message_t tx;
    tx.data_length_code = 8; // Always
    GS_338 gs_338tx;
    GS_218 gs_218tx;
    GS_418 gs_418tx;
    GS_CUSTOM_558 gs_558tx;
    GS_CUSTOM_668 gs_668tx;

    // Copy current CAN frame values to here so we don't
    // accidentally modify parity calculations
    gs_338tx = {gs338.raw};
    gs_218tx = {gs218.raw};
    gs_418tx = {gs418.raw};
    gs_558tx = {gs558.raw};
    gs_668tx = {gs668.raw};

    // Firstly we have to deal with toggled bits!
    // As toggle bits need to be toggled every 40ms,
    // and egs52 Tx interval is 20ms,
    // we can achieve this with 2 booleans
    gs_218tx.set_MTGL_EGS(toggle);
    gs_418tx.set_FMRADTGL(toggle);
    // Now do parity calculations
    gs_218tx.set_MPAR_EGS(calc_torque_parity(gs_218tx.raw >> 48));
    // Includes FMRAD and FMRADTGL signal in this mask
    gs_418tx.set_FMRADPAR(calc_torque_parity(gs_418tx.raw & 0x47FF));
    if (time_to_toggle) {
        toggle = !toggle;
    }
    time_to_toggle = !time_to_toggle;
    
    // Now set CVN Counter (Increases every frame)
    gs_218tx.set_FEHLER(cvn_counter);
    cvn_counter++;

    // Now send CAN Data!

    /**
     * TX order of EGS52:
     * GS_338
     * GS_218
     * GS_418
     * GS_CUSTOM_558 ;)
     */
    tx.identifier = GS_338_CAN_ID;
    to_bytes(gs_338tx.raw, tx.data);
    twai_transmit(&tx, 5);
    tx.identifier = GS_218_CAN_ID;
    to_bytes(gs_218tx.raw, tx.data);
    twai_transmit(&tx, 5);
    tx.identifier = GS_418_CAN_ID;
    to_bytes(gs_418tx.raw, tx.data);
    twai_transmit(&tx, 5);
    tx.identifier = GS_CUSTOM_558_CAN_ID;
    to_bytes(gs_558tx.raw, tx.data);
    twai_transmit(&tx, 5);
    tx.identifier = GS_CUSTOM_668_CAN_ID;
    to_bytes(gs_668tx.raw, tx.data);
    twai_transmit(&tx, 5);
}

void Egs52Can::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) {
    if(this->ecu_ms.import_frames(data, id, timestamp)) {
    } else if (this->esp_ecu.import_frames(data, id, timestamp)) {
    } else if (this->ewm_ecu.import_frames(data, id, timestamp)) {
    } else if (this->misc_ecu.import_frames(data, id, timestamp)) {
    } else if (this->ezs_ecu.import_frames(data, id, timestamp)) {
    }
}