#include "can_egs53.h"
#include "driver/twai.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"

uint8_t crcTable[256]; // For CRC only

Egs53Can::Egs53Can(const char* name, uint8_t tx_time_ms, uint32_t baud) : EgsBaseCan(name, tx_time_ms, baud){
    // Create CRC table
    for (int i = 0; i < 0x100; i++) {
            uint8_t _crc = i;
            for (uint8_t bit = 0; bit < 8; bit++) {
                _crc = (_crc & 0x80) ? ((_crc << 1) ^ 0x1D) : (_crc << 1);
            }
            crcTable[i] = _crc;
    }
    // Set default values
    this->sbw_rs_tcm.SBW_MsgTxmtId = SBW_RS_TCM_SBW_MsgTxmtId_EGS53::EGS52; // We are EGS53
    this->sbw_rs_tcm.TSL_Posn_Rq = SBW_RS_TCM_TSL_Posn_Rq_EGS53::IDLE; // Idle request (No SBW on EGS53)
    this->sbw_rs_tcm.TxSelSensPosn = 0; // No dialing sensor on EGS53
    // Tell engine which Mech style we are
    if (VEHICLE_CONFIG.is_large_nag != 0) {
        this->eng_rq2_tcm.TxMechStyle = ENG_RQ2_TCM_TxMechStyle_EGS53::LARGE;
    } else {
        this->eng_rq2_tcm.TxMechStyle = ENG_RQ2_TCM_TxMechStyle_EGS53::SMALL;
    }
    this->eng_rq2_tcm.TxStyle = ENG_RQ2_TCM_TxStyle_EGS53::SAT; // Stepped automatic gearbox
    this->eng_rq2_tcm.TxShiftStyle = ENG_RQ2_TCM_TxShiftStyle_EGS53::MS; // Mechanical shifting (With EWM module)
}

WheelData Egs53Can::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    WHL_STAT2_EGS53 whl_stat;
    if (this->ecm_ecu.get_WHL_STAT2(now, expire_time_ms*1000, &whl_stat)) {
        WheelDirection dir;
        switch(whl_stat.WhlDir_FR_Stat) {
            case WHL_STAT2_WhlDir_FR_Stat_EGS53::VOID:
                dir = WheelDirection::Stationary;
                break;
            case WHL_STAT2_WhlDir_FR_Stat_EGS53::FORWARD:
                dir = WheelDirection::Forward;
                break;
            case WHL_STAT2_WhlDir_FR_Stat_EGS53::BACKWARD:
                dir = WheelDirection::Reverse;
                break;
            default:
                dir = WheelDirection::SignalNotAvailable;
                break;
        }
        return WheelData {
            .double_rpm = whl_stat.WhlRPM_FR,
            .current_dir = dir
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

WheelData Egs53Can::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    WHL_STAT2_EGS53 whl_stat;
    if (this->ecm_ecu.get_WHL_STAT2(now, expire_time_ms*1000, &whl_stat)) {
        WheelDirection dir;
        switch(whl_stat.WhlDir_FL_Stat) {
            case WHL_STAT2_WhlDir_FL_Stat_EGS53::VOID:
                dir = WheelDirection::Stationary;
                break;
            case WHL_STAT2_WhlDir_FL_Stat_EGS53::FORWARD:
                dir = WheelDirection::Forward;
                break;
            case WHL_STAT2_WhlDir_FL_Stat_EGS53::BACKWARD:
                dir = WheelDirection::Reverse;
                break;
            default:
                dir = WheelDirection::SignalNotAvailable;
                break;
        }
        return WheelData {
            .double_rpm = whl_stat.WhlRPM_FL,
            .current_dir = dir
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

WheelData Egs53Can::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    WHL_STAT2_EGS53 whl_stat;
    if (this->ecm_ecu.get_WHL_STAT2(now, expire_time_ms*1000, &whl_stat)) {
        WheelDirection dir;
        switch(whl_stat.WhlDir_RR_Stat) {
            case WHL_STAT2_WhlDir_RR_Stat_EGS53::VOID:
                dir = WheelDirection::Stationary;
                break;
            case WHL_STAT2_WhlDir_RR_Stat_EGS53::FORWARD:
                dir = WheelDirection::Forward;
                break;
            case WHL_STAT2_WhlDir_RR_Stat_EGS53::BACKWARD:
                dir = WheelDirection::Reverse;
                break;
            default:
                dir = WheelDirection::SignalNotAvailable;
                break;
        }
        return WheelData {
            .double_rpm = whl_stat.WhlRPM_RR,
            .current_dir = dir
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

WheelData Egs53Can::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    WHL_STAT2_EGS53 whl_stat;
    if (this->ecm_ecu.get_WHL_STAT2(now, expire_time_ms*1000, &whl_stat)) {
        WheelDirection dir;
        switch(whl_stat.WhlDir_RL_Stat) {
            case WHL_STAT2_WhlDir_RL_Stat_EGS53::VOID:
                dir = WheelDirection::Stationary;
                break;
            case WHL_STAT2_WhlDir_RL_Stat_EGS53::FORWARD:
                dir = WheelDirection::Forward;
                break;
            case WHL_STAT2_WhlDir_RL_Stat_EGS53::BACKWARD:
                dir = WheelDirection::Reverse;
                break;
            default:
                dir = WheelDirection::SignalNotAvailable;
                break;
        }
        return WheelData {
            .double_rpm = whl_stat.WhlRPM_RL,
            .current_dir = dir
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

ShifterPosition Egs53Can::get_shifter_position(uint64_t now, uint64_t expire_time_ms) {
    SBW_RS_ISM_EGS53 tslm;
    if (this->tslm_ecu.get_SBW_RS_ISM(now, expire_time_ms*1000, &tslm)) {
        switch (tslm.TSL_Posn_ISM) {
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::D:
                return ShifterPosition::D;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::N:
                return ShifterPosition::N;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::R:
                return ShifterPosition::R;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::P:
                return ShifterPosition::P;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::PLUS:
                return ShifterPosition::PLUS;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::MINUS:
                return ShifterPosition::MINUS;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::N_ZW_D:
                return ShifterPosition::N_D;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::R_ZW_N:
                return ShifterPosition::R_N;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::P_ZW_R:
                return ShifterPosition::P_R;
            case SBW_RS_ISM_TSL_Posn_ISM_EGS53::SNA:
            default:
                return ShifterPosition::SignalNotAvailable;
        }
    }
    return ShifterPosition::SignalNotAvailable;
}

EngineType Egs53Can::get_engine_type(uint64_t now, uint64_t expire_time_ms) {
    return EngineType::Unknown;
}

bool Egs53Can::get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs53Can::get_kickdown(uint64_t now, uint64_t expire_time_ms) { // TODO
    ENG_RS3_PT_EGS53 eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(now, expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.KickDnSw_Psd;
    }
    return false;
}

uint8_t Egs53Can::get_pedal_value(uint64_t now, uint64_t expire_time_ms) {
    ENG_RS3_PT_EGS53 eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(now, expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.AccelPdlPosn_Raw; // Use RAW position, not 'modified' value from ECM!
    }
    return 0;
}

int Egs53Can::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    ENG_RS2_PT_EGS53 rs2_pt;
    if (this->ecm_ecu.get_ENG_RS2_PT(now, expire_time_ms*1000, &rs2_pt)) {
        return (rs2_pt.EngTrqStatic / 4) - 500;
    }
    return 0;
}

int Egs53Can::get_driver_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    return this->get_static_engine_torque(now, expire_time_ms);
}

int Egs53Can::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    ENG_RS2_PT_EGS53 rs2_pt;
    if (this->ecm_ecu.get_ENG_RS2_PT(now, expire_time_ms*1000, &rs2_pt)) {
        return (rs2_pt.EngTrqMaxETC / 4) - 500;
    }
    return 0;
}

int Egs53Can::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    ENG_RS2_PT_EGS53 rs2_pt;
    if (this->ecm_ecu.get_ENG_RS2_PT(now, expire_time_ms*1000, &rs2_pt)) {
        return (rs2_pt.EngTrqMinTTC / 4) - 500;
    }
    return 0;
}

PaddlePosition Egs53Can::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    SBW_RQ_SCCM_EGS53 sbw_rq;
    PaddlePosition ret = PaddlePosition::SNV;
    if (this->ecm_ecu.get_SBW_RQ_SCCM(now, expire_time_ms*1000, &sbw_rq)) {
        switch(sbw_rq.StW_Sw_Stat3) {
            case SBW_RQ_SCCM_StW_Sw_Stat3_EGS53::MINUS: // Minus
                ret = PaddlePosition::Minus;
                break;
            case SBW_RQ_SCCM_StW_Sw_Stat3_EGS53::PLUS: // Plus
                ret = PaddlePosition::Plus;
                break;
            case SBW_RQ_SCCM_StW_Sw_Stat3_EGS53::PLUS_MINUS: // Plus + Minus
                ret = PaddlePosition::PlusAndMinus;
                break;
            case SBW_RQ_SCCM_StW_Sw_Stat3_EGS53::NPSD: // None
                ret = PaddlePosition::None;
                break;
            default: // Other??
                break;
        }
    }
    return ret;
}

int16_t Egs53Can::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    ECM_A1_EGS53 ecm_a1;
    uint16_t res = INT16_MAX;
    if (this->ecm_ecu.get_ECM_A1(now, expire_time_ms*1000, &ecm_a1)) {
        if (ecm_a1.EngCoolTemp != UINT8_MAX) {
            res = ecm_a1.EngCoolTemp - 40;
        }
    }
    return res;
}

int16_t Egs53Can::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) { // TODO
    ECM_A1_EGS53 ecm_a1;
    uint16_t res = INT16_MAX;
    if (this->ecm_ecu.get_ECM_A1(now, expire_time_ms*1000, &ecm_a1)) {
        if (ecm_a1.EngOilTemp != UINT8_MAX) {
            res = ecm_a1.EngOilTemp - 40;
        }
    }
    return res;
}

int16_t Egs53Can::get_engine_iat_temp(uint64_t now, uint64_t expire_time_ms) {
    ECM_A1_EGS53 ecm_a1;
    uint16_t res = INT16_MAX;
    if (this->ecm_ecu.get_ECM_A1(now, expire_time_ms*1000, &ecm_a1)) {
        if (ecm_a1.IntkAirTemp != UINT8_MAX) {
            res = ecm_a1.IntkAirTemp - 40;
        }
    }
    return res;
}

uint16_t Egs53Can::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    ENG_RS3_PT_EGS53 eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(now, expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.EngRPM;
    }
    return UINT16_MAX; // UNDEFINED
}

bool Egs53Can::get_is_starting(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs53Can::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    SBW_RS_ISM_EGS53 tslm;
    if (this->tslm_ecu.get_SBW_RS_ISM(now, expire_time_ms*10000, &tslm)) {
        return tslm.TxDrvProgSw_Psd_V3;
    }
    return false;
}

bool Egs53Can::get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

void Egs53Can::set_clutch_status(ClutchStatus status) {
    
}

void Egs53Can::set_actual_gear(GearboxGear actual) {
    switch (actual) {
        case GearboxGear::Park:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::P;
            break;
        case GearboxGear::Reverse_First:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::R;
            break;
        case GearboxGear::Reverse_Second:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::R_2;
            break;
        case GearboxGear::Neutral:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::N;
            break;
        case GearboxGear::First:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::D1;
            break;
        case GearboxGear::Second:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::D2;
            break;
        case GearboxGear::Third:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::D3;
            break;
        case GearboxGear::Fourth:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::D4;
            break;
        case GearboxGear::Fifth:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::D5;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->eng_rq2_tcm.Gr = ENG_RQ2_TCM_Gr_EGS53::SNA;
            break;
    }
}

void Egs53Can::set_target_gear(GearboxGear target) {
    switch (target) {
        case GearboxGear::Park:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::P;
            break;
        case GearboxGear::Reverse_First:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::R;
            break;
        case GearboxGear::Reverse_Second:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::R_2;
            break;
        case GearboxGear::Neutral:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::N;
            break;
        case GearboxGear::First:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::D1;
            break;
        case GearboxGear::Second:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::D2;
            break;
        case GearboxGear::Third:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::D3;
            break;
        case GearboxGear::Fourth:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::D4;
            break;
        case GearboxGear::Fifth:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::D5;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->eng_rq2_tcm.Gr_Target = ENG_RQ2_TCM_Gr_Target_EGS53::SNA;
            break;
    }
}

void Egs53Can::set_safe_start(bool can_start) {
    this->sbw_rs_tcm.StartLkSw = can_start;
    this->eng_rq1_tcm.EngSt_Enbl_Rq_TCM = can_start;
}

void Egs53Can::set_gearbox_temperature(uint16_t temp) {
    this->tcm_a1.TxOilTemp = temp + 50;
}

void Egs53Can::set_input_shaft_speed(uint16_t rpm) {
    this->tcm_a2.TxTurbineRPM = rpm;
}

void Egs53Can::set_is_all_wheel_drive(bool is_4wd) {
    
}

void Egs53Can::set_wheel_torque(uint16_t t) {

}

void Egs53Can::set_shifter_position(ShifterPosition pos) {
    switch (pos) {
        case ShifterPosition::P:
            this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::P;
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::P;
            break;
        case ShifterPosition::P_R:
            this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::P_ZW_R;
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::P;
            break;
        case ShifterPosition::R:
            this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::R;
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::R;
            break;
        case ShifterPosition::R_N:
            this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::R_ZW_N;
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::R;
            break;
        case ShifterPosition::N:
            this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::N;
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::N;
            break;
        case ShifterPosition::N_D:
            this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::N_ZW_D;
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::N;
            break;
        case ShifterPosition::D:
        case ShifterPosition::PLUS:
        case ShifterPosition::MINUS:
            this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::D;
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::D;
            break;
        case ShifterPosition::SignalNotAvailable:
        default:
            this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::SNA;
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::SNA;
            break;
    }
}

void Egs53Can::set_gearbox_ok(bool is_ok) {
    this->tcm_a1.TCM_LHOM = !is_ok;
    this->tcm_a1.BasShftProg_Ok = is_ok;
}

void Egs53Can::set_torque_request(TorqueRequest request, float amount_nm) {

}

void Egs53Can::set_error_check_status(SystemStatusCheck ssc) {
    
}


void Egs53Can::set_turbine_torque_loss(uint16_t loss_nm) {
    
}

// uint8_t x = 0;
unsigned long last_time = 0;
void Egs53Can::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
    switch (g) {
        case GearboxDisplayGear::One:
            manual_mode ? this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::M1
            : this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::D1;
            break;
        case GearboxDisplayGear::Two:
            manual_mode ? this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::M2
            : this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::D2;
            break;
        case GearboxDisplayGear::Three:
            manual_mode ? this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::M3
            : this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::D3;
            break;
        case GearboxDisplayGear::Four:
            manual_mode ? this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::M4
            : this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::D4;
            break;
        case GearboxDisplayGear::Five:
            manual_mode ? this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::M1
            : this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::D1;
            break;
        case GearboxDisplayGear::P:
            this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::P;
            break;
        case GearboxDisplayGear::D:
            this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::D;
            break;
        case GearboxDisplayGear::N:
            this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::N;
            break;
        case GearboxDisplayGear::R:
            this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::R;
            break;
        case GearboxDisplayGear::Failure:
            this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::F;
            break;
        case GearboxDisplayGear::SNA:
        default:
            this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::BLANK;
            break;
    }
}

void Egs53Can::set_drive_profile(GearboxProfile p) {
    switch(p) {
        case GearboxProfile::Agility:
            this->tcm_disp_rq.TxDrvProg_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM_EGS53::A;
            break;
        case GearboxProfile::Comfort:
            this->tcm_disp_rq.TxDrvProg_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM_EGS53::C;
            break;
        case GearboxProfile::Standard:
            this->tcm_disp_rq.TxDrvProg_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM_EGS53::S;
            break;
        case GearboxProfile::Winter:
            this->tcm_disp_rq.TxDrvProg_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM_EGS53::W;
            break;
        case GearboxProfile::Manual:
            this->tcm_disp_rq.TxDrvProg_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM_EGS53::M;
            break;
        case GearboxProfile::Failure:
            this->tcm_disp_rq.TxDrvProg_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM_EGS53::F;
            break;
        default:
            this->tcm_disp_rq.TxDrvProg_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM_EGS53::BLANK;
            break;
    }
}

void Egs53Can::set_display_msg(GearboxMessage msg) {
    
}

void Egs53Can::set_wheel_torque_multi_factor(float ratio) {
    eng_rq2_tcm.EngWhlTrqRatio_TCM = ratio * 100;
}

/**
 * Calculates CRC based on SAE-J180.
 * As all the CAN Frames work the same, we always assume its the first 7 bytes, and result is to be placed
 * in the 8th byte in the frame
 */
void calc_crc_in_place(uint8_t* buffer) {
    // assume len = 7
    unsigned long crc;
    int i;
    int bit;
    
    crc = 0xFF;
    for ( i=0 ; i<7 ; i++ ) {
        crc ^= buffer[i];
        for ( bit=0 ; bit<8 ; bit++ ) {
            if ( (crc & 0x80)!=0 ) {
                crc <<= 1;
                crc ^= 0x1D;
            }
            else {
                crc <<= 1;
            }
        }
    }

    buffer[7] = (~crc)&0xFF;
}

inline void to_bytes(uint64_t src, uint8_t* dst) {
    for(uint8_t i = 0; i < 8; i++) {
        dst[7-i] = src & 0xFF;
        src >>= 8;
    }
}

uint8_t msg_counter = 0;


void Egs53Can::tx_frames() {
    tx.data_length_code = 8; // Always
    TCM_A1_EGS53 tcm_a1_tx = {0};
    TCM_A2_EGS53 tcm_a2_tx = {0};
    ENG_RQ1_TCM_EGS53 eng_rq1_tcm_tx = {0};
    ENG_RQ2_TCM_EGS53 eng_rq2_tcm_tx = {0};
    ENG_RQ3_TCM_EGS53 eng_rq3_tcm_tx = {0};
    SBW_RS_TCM_EGS53 sbw_rs_tcm_tx = {0};
    TCM_DISP_RQ_EGS53 tcm_disp_rq_tx = {0};
    NM_TCM_EGS53 nm_tcm_tx = {0};

    // Copy current CAN frame values to here so we don't
    // accidentally modify parity calculations
    tcm_a1_tx = {tcm_a1.raw};
    tcm_a2_tx = {tcm_a2.raw};
    eng_rq1_tcm_tx = {eng_rq1_tcm.raw};
    eng_rq2_tcm_tx = {eng_rq2_tcm.raw};
    eng_rq3_tcm_tx = {eng_rq3_tcm.raw};
    sbw_rs_tcm_tx = {sbw_rs_tcm.raw};
    tcm_disp_rq_tx = {tcm_disp_rq.raw};
    nm_tcm_tx = {nm_tcm.raw};

    tcm_a2_tx.TCM_CALID_CVN_ErrNum = cvn_counter;
    cvn_counter++;
    if (cvn_counter == 0x14) {
        cvn_counter = 0;
    }

    // Set message counters
    eng_rq1_tcm_tx.MC_ENG_RQ1_TCM = msg_counter;
    eng_rq2_tcm_tx.MC_ENG_RQ2_TCM = msg_counter;
    eng_rq3_tcm_tx.MC_ENG_RQ3_TCM = msg_counter;
    sbw_rs_tcm_tx.MC_SBW_RS_TCM = msg_counter;

    msg_counter++; // Global for all messages out of TCM
    // Now send CAN Data!
    tx.identifier = ENG_RQ1_TCM_EGS53_CAN_ID;
    to_bytes(eng_rq1_tcm_tx.raw, tx.data);
    calc_crc_in_place(tx.data);
    twai_transmit(&tx, 5);
    
    tx.identifier = ENG_RQ2_TCM_EGS53_CAN_ID;
    to_bytes(eng_rq2_tcm_tx.raw, tx.data);  
    calc_crc_in_place(tx.data);
    twai_transmit(&tx, 5);

    tx.identifier = TCM_A2_EGS53_CAN_ID;
    to_bytes(tcm_a2_tx.raw, tx.data);
    calc_crc_in_place(tx.data);
    twai_transmit(&tx, 5);

    tx.identifier = TCM_A1_EGS53_CAN_ID;
    to_bytes(tcm_a1_tx.raw, tx.data);
    twai_transmit(&tx, 5);

    if (counter == 5) {
        tx.identifier = TCM_DISP_RQ_EGS53_CAN_ID;
        to_bytes(tcm_disp_rq_tx.raw, tx.data);
        twai_transmit(&tx, 5);
        counter = 0;
    }
    counter++;

    tx.identifier = SBW_RS_TCM_EGS53_CAN_ID;
    to_bytes(sbw_rs_tcm_tx.raw, tx.data);
    calc_crc_in_place(tx.data);
    twai_transmit(&tx, 5);
}

void Egs53Can::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) {
    if(this->ecm_ecu.import_frames(data, id, timestamp)) {
    } else if (this->fscm_ecu.import_frames(data, id, timestamp)) {
    } else if (this->tslm_ecu.import_frames(data, id, timestamp)) {
    }
}