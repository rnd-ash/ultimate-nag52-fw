#include "can_egs53.h"
#include "driver/twai.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"
#include "egs_calibration/calibration_structs.h"

uint8_t crcTable[256]; // For CRC only

Egs53Can::Egs53Can(const char *name, uint8_t tx_time_ms, uint32_t baud, Shifter *shifter) : EgsBaseCan(name, tx_time_ms, baud, shifter){
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
    this->sbw_rs_tcm.TxSelSensPosn = 0xFF; // We never report this
    this->sbw_rs_tcm.TxSelVlvPosn = SBW_RS_TCM_TxSelVlvPosn_EGS53::SNA; // We never report this
    this->sbw_rs_tcm.bytes[4] = 0x00;

    // Tell engine which Mech style we are
    if (MECH_PTR->gb_ty == 0) {
        this->eng_rq2_tcm.TxMechStyle = ENG_RQ2_TCM_TxMechStyle_EGS53::LARGE;
    } else {
        this->eng_rq2_tcm.TxMechStyle = ENG_RQ2_TCM_TxMechStyle_EGS53::SMALL;
    }
    this->eng_rq2_tcm.TxStyle = ENG_RQ2_TCM_TxStyle_EGS53::SAT; // Stepped automatic gearbox
    this->eng_rq2_tcm.TxShiftStyle = ENG_RQ2_TCM_TxShiftStyle_EGS53::MS; // Mechanical shifting (With EWM module)
    
    this->tcm_a1.Clutch_Stat = TCM_A1_Clutch_Stat_EGS53::DISENGG;
    this->tcm_a2.CurrDtyCyc_Rq = 0xFF;
    this->tcm_a2.TxSlpRPM_Dsr = 0x3FFF;
    this->tcm_a2.TCM_Data = 0xFF;
    this->tcm_a2.TCM_ErrChk_Stat = TCM_A2_TCM_ErrChk_Stat_EGS53::OK;
    this->tcm_a2.TCM_CALID_CVN_Actv = 0;
}

uint16_t Egs53Can::get_front_right_wheel(const uint32_t expire_time_ms)
{
	WHL_STAT2_EGS53 whl_stat;
    uint16_t ret = UINT16_MAX;
    if (this->ecm_ecu.get_WHL_STAT2(GET_CLOCK_TIME(), expire_time_ms*1000, &whl_stat)) {
        if (whl_stat.WhlDir_FR_Stat != WHL_STAT2_WhlDir_FR_Stat_EGS53::SNA) {
            ret = whl_stat.WhlRPM_FR;
        }
    }
    return ret;
}

uint16_t Egs53Can::get_front_left_wheel(const uint32_t expire_time_ms) { // TODO
    WHL_STAT2_EGS53 whl_stat;
    uint16_t ret = UINT16_MAX;
    if (this->ecm_ecu.get_WHL_STAT2(GET_CLOCK_TIME(), expire_time_ms*1000, &whl_stat)) {
        if (whl_stat.WhlDir_FL_Stat != WHL_STAT2_WhlDir_FL_Stat_EGS53::SNA) {
            ret = whl_stat.WhlRPM_FL;
        }
    }
    return ret;
}

uint16_t Egs53Can::get_rear_right_wheel(const uint32_t expire_time_ms) {
    WHL_STAT2_EGS53 whl_stat;
    uint16_t ret = UINT16_MAX;
    if (this->ecm_ecu.get_WHL_STAT2(GET_CLOCK_TIME(), expire_time_ms*1000, &whl_stat)) {
        if (whl_stat.WhlDir_RR_Stat != WHL_STAT2_WhlDir_RR_Stat_EGS53::SNA) {
            ret = whl_stat.WhlRPM_RR;
        }
    }
    return ret;
}

uint16_t Egs53Can::get_rear_left_wheel(const uint32_t expire_time_ms) {
    WHL_STAT2_EGS53 whl_stat;
    uint16_t ret = UINT16_MAX;
    if (this->ecm_ecu.get_WHL_STAT2(GET_CLOCK_TIME(), expire_time_ms*1000, &whl_stat)) {
        if (whl_stat.WhlDir_RL_Stat != WHL_STAT2_WhlDir_RL_Stat_EGS53::SNA) {
            ret = whl_stat.WhlRPM_RL;
        }
    }
    return ret;
}

ShifterPosition Egs53Can::internal_can_shifter_get_shifter_position(const uint32_t expire_time_ms) {
    SBW_RS_ISM_EGS53 tslm;
    if (this->tslm_ecu.get_SBW_RS_ISM(GET_CLOCK_TIME(), expire_time_ms*1000, &tslm)) {
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

EngineType Egs53Can::get_engine_type(const uint32_t expire_time_ms) {
    return EngineType::Unknown;
}

bool Egs53Can::get_engine_is_limp(const uint32_t expire_time_ms) { // TODO
    return false;
}

bool Egs53Can::get_kickdown(const uint32_t expire_time_ms) { // TODO
    ENG_RS3_PT_EGS53 eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(GET_CLOCK_TIME(), expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.KickDnSw_Psd;
    }
    return false;
}

uint8_t Egs53Can::get_pedal_value(const uint32_t expire_time_ms) {
    ENG_RS3_PT_EGS53 eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(GET_CLOCK_TIME(), expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.AccelPdlPosn_Raw; // Use RAW position, not 'modified' value from ECM!
    }
    return 0;
}

CanTorqueData Egs53Can::get_torque_data(const uint32_t expire_time_ms) {
    ENG_RS2_PT_EGS53 rs2_pt;
    ENG_RS1_PT_EGS53 rs1_pt;
    int16_t sta = INT16_MAX;
    int16_t esp = INT16_MAX;
    CanTorqueData ret = TORQUE_NDEF;
    if (
        this->ecm_ecu.get_ENG_RS2_PT(GET_CLOCK_TIME(), expire_time_ms, &rs2_pt) &&
        this->ecm_ecu.get_ENG_RS1_PT(GET_CLOCK_TIME(), expire_time_ms, &rs1_pt)
    ) {
        if (rs2_pt.EngTrqStatic != INT16_MAX) {
            sta = (rs2_pt.EngTrqStatic / 4) - 500;
        }
        if (rs2_pt.EngTrqMaxETC != INT16_MAX) {
            ret.m_max = (rs2_pt.EngTrqMaxETC / 4) - 500;
        }
        if (rs2_pt.EngTrqMinTTC != INT16_MAX) {
            ret.m_min = (rs2_pt.EngTrqMinTTC / 4) - 500;
        }
        if (rs1_pt.EngTrqSel_D_TTC != INT16_MAX) {
            esp = (rs1_pt.EngTrqSel_D_TTC / 4) - 500;
        }
    }
    if (INT16_MAX != sta && INT16_MAX != esp) {
        // Conversion
        int static_converted = sta;
        int tmp = esp;
        int driver_converted = static_converted;
        int indicated = 0;
        // Calculate converted torque from ESP
        // Chrysler cars don't seem to report MAX/MIN
        if (INT_MAX != ret.m_max && INT_MAX != ret.m_min) {
            tmp = MIN(esp, ret.m_max);
        }
        if (tmp <= 0) {
            tmp = MIN(tmp, static_converted);
        }
        driver_converted = tmp;

        // Check if freezing torque should be done
        bool active_shift = (uint8_t)this->eng_rq2_tcm.Gr_Target != (uint8_t)this->eng_rq2_tcm.Gr;
        bool trq_req_en = this->eng_rq1_tcm.EngTrqMin_Rq_TCM != 0 || this->eng_rq1_tcm.EngTrqMax_Rq_TCM != 0;
        if (active_shift && trq_req_en) {
            this->freeze_torque = true; // Gear shift and we have started a torque request, freeze it
        } else if (!active_shift) {
            this->freeze_torque = false; // No gear shift, unfreeze it
        }
        // Change torque values based on freezing or not
        if (this->freeze_torque) {
            driver_converted = MAX(driver_converted - this->req_static_torque_delta, static_converted);
        } else {
            this->req_static_torque_delta = driver_converted - static_converted;
        }
        if (driver_converted > 0) {
            indicated = driver_converted;
        }
        ret.m_ind = indicated;
        ret.m_converted_driver = driver_converted;
        ret.m_converted_static = static_converted;

    }
    return ret;
}

PaddlePosition Egs53Can::get_paddle_position(const uint32_t expire_time_ms) {
    SBW_RQ_SCCM_EGS53 sbw_rq;
    PaddlePosition ret = PaddlePosition::SNV;
    if (this->ecm_ecu.get_SBW_RQ_SCCM(GET_CLOCK_TIME(), expire_time_ms*1000, &sbw_rq)) {
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

int16_t Egs53Can::get_engine_coolant_temp(const uint32_t expire_time_ms) {
    ECM_A1_EGS53 ecm_a1;
    uint16_t res = INT16_MAX;
    if (this->ecm_ecu.get_ECM_A1(GET_CLOCK_TIME(), expire_time_ms*1000, &ecm_a1)) {
        if (ecm_a1.EngCoolTemp != UINT8_MAX) {
            res = ecm_a1.EngCoolTemp - 40;
        }
    }
    return res;
}

int16_t Egs53Can::get_engine_oil_temp(const uint32_t expire_time_ms) { // TODO
    ECM_A1_EGS53 ecm_a1;
    uint16_t res = INT16_MAX;
    if (this->ecm_ecu.get_ECM_A1(GET_CLOCK_TIME(), expire_time_ms*1000, &ecm_a1)) {
        if (ecm_a1.EngOilTemp != UINT8_MAX) {
            res = ecm_a1.EngOilTemp - 40;
        }
    }
    return res;
}

int16_t Egs53Can::get_engine_iat_temp(const uint32_t expire_time_ms) {
    ECM_A1_EGS53 ecm_a1;
    uint16_t res = INT16_MAX;
    if (this->ecm_ecu.get_ECM_A1(GET_CLOCK_TIME(), expire_time_ms*1000, &ecm_a1)) {
        if (ecm_a1.IntkAirTemp != UINT8_MAX) {
            res = ecm_a1.IntkAirTemp - 40;
        }
    }
    return res;
}

uint16_t Egs53Can::get_engine_rpm(const uint32_t expire_time_ms) {
    ENG_RS3_PT_EGS53 eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(GET_CLOCK_TIME(), expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.EngRPM;
    }
    return UINT16_MAX; // UNDEFINED
}

bool Egs53Can::get_is_starting(const uint32_t expire_time_ms) { // TODO
    return false;
}

bool Egs53Can::get_profile_btn_press(const uint32_t expire_time_ms) {
    SBW_RS_ISM_EGS53 tslm;
    if (this->tslm_ecu.get_SBW_RS_ISM(GET_CLOCK_TIME(), expire_time_ms*10000, &tslm)) {
        return tslm.TxDrvProgSw_Psd_V3;
    }
    return false;
}

bool Egs53Can::get_is_brake_pressed(const uint32_t expire_time_ms) {
    BRK_STAT_EGS53 brk;
    bool ret = false;
    if (this->ecm_ecu.get_BRK_STAT(GET_CLOCK_TIME(), expire_time_ms, &brk)) {
        ret = brk.Brk_Stat == BRK_STAT_Brk_Stat_EGS53::BRAKING;
    }
    return ret;
}

bool Egs53Can::engine_ack_torque_request(const uint32_t expire_time_ms) {
    ENG_RS1_PT_EGS53 engrs1;
    bool ret = false;
    if (this->ecm_ecu.get_ENG_RS1_PT(GET_CLOCK_TIME(), expire_time_ms, &engrs1)) {
        ret = engrs1.EngTrq_Ack_ECM;
    }
    return ret;
}

bool Egs53Can::esp_torque_intervention_active(const uint32_t expire_time_ms) {
    return false; // TODO
}

bool Egs53Can::is_cruise_control_active(const uint32_t expire_time_ms) {
    return false; // TODO
}

int Egs53Can::cruise_control_torque_demand(const uint32_t expire_time_ms) {
    return INT_MAX; // TODO
}

int Egs53Can::esp_torque_demand(const uint32_t expire_time_ms) {
    return INT_MAX; // TODO
}

TccReqState Egs53Can::get_engine_tcc_override_request(const uint32_t expire_time_ms) {
    TX_RQ_ECM_EGS53 tx_rq_ecm;
    TccReqState ret = TccReqState::None;
    // Check for slip first
    if (this->ecm_ecu.get_TX_RQ_ECM(GET_CLOCK_TIME(), expire_time_ms, &tx_rq_ecm)) {
        switch (tx_rq_ecm.TCC_Rq) {
            case TX_RQ_ECM_TCC_Rq_EGS53::ENGG:
                ret = TccReqState::Slipping;
                break;
            case TX_RQ_ECM_TCC_Rq_EGS53::DISENGG:
                ret = TccReqState::Open;
                break;
            default:
                break;
        }
    }
    return ret;
}

void Egs53Can::set_clutch_status(TccClutchStatus status) {
    
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

void Egs53Can::set_gearbox_temperature(int16_t temp) {
    this->tcm_a1.TxOilTemp = MAX(temp, -50) + 50;
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
            this->sbw_rs_tcm.TSL_Posn_Rq = SBW_RS_TCM_TSL_Posn_Rq_EGS53::P;
            break;
        case ShifterPosition::P_R:
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::P;
            break;
        case ShifterPosition::R:
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::R;
            break;
        case ShifterPosition::R_N:
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::R;
            break;
        case ShifterPosition::N:
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::N;
            break;
        case ShifterPosition::N_D:
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::N;
            break;
        case ShifterPosition::D:
        case ShifterPosition::PLUS:
        case ShifterPosition::MINUS:
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::D;
            break;
        case ShifterPosition::SignalNotAvailable:
        default:
            this->tcm_a1.TSL_Posn_TCM = TCM_A1_TSL_Posn_TCM_EGS53::SNA;
            break;
    }
}

void Egs53Can::set_gearbox_ok(bool is_ok) {
    this->tcm_a1.TCM_LHOM = !is_ok;
    this->tcm_a1.BasShftProg_Ok = is_ok;
}

void Egs53Can::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    if (control_type == TorqueRequestControlType::None) {
        eng_rq1_tcm.IntrvntnMd_TCM = ENG_RQ1_TCM_IntrvntnMd_TCM_EGS53::MFC;
    } else if (control_type == TorqueRequestControlType::FastAsPossible) {
        eng_rq1_tcm.IntrvntnMd_TCM = ENG_RQ1_TCM_IntrvntnMd_TCM_EGS53::FAST;
    } else if (control_type == TorqueRequestControlType::BackToDemandTorque) {
        eng_rq1_tcm.IntrvntnMd_TCM = ENG_RQ1_TCM_IntrvntnMd_TCM_EGS53::RATE_INC;
    } else { // Normal speed
        eng_rq1_tcm.IntrvntnMd_TCM = ENG_RQ1_TCM_IntrvntnMd_TCM_EGS53::MFC;
    }

    if (control_type != TorqueRequestControlType::None) {
        eng_rq1_tcm.EngTrq_Rq_TCM = (amount_nm + 500) * 4;
        if (limit_type == TorqueRequestBounds::LessThan) {
            eng_rq1_tcm.EngTrqMin_Rq_TCM = true;
            eng_rq1_tcm.EngTrqMax_Rq_TCM = false;
        } else if (limit_type == TorqueRequestBounds::MoreThan) {
            eng_rq1_tcm.EngTrqMin_Rq_TCM = false;
            eng_rq1_tcm.EngTrqMax_Rq_TCM = true;
        } else {
            eng_rq1_tcm.EngTrqMin_Rq_TCM = true;
            eng_rq1_tcm.EngTrqMax_Rq_TCM = true;
        }
    } else {
        // no intervention
        eng_rq1_tcm.EngTrqMin_Rq_TCM = false;
        eng_rq1_tcm.EngTrqMax_Rq_TCM = false;
        eng_rq1_tcm.EngTrq_Rq_TCM = 0;
    }
}

void Egs53Can::set_error_check_status(SystemStatusCheck ssc) {
    
}


void Egs53Can::set_turbine_torque_loss(uint16_t loss_nm) {
    
}

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
            manual_mode ? this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::M5
            : this->tcm_disp_rq.TxDrvPosn_Disp_Rq_TCM = TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM_EGS53::D5;
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
    if (ratio == -1) {
        eng_rq2_tcm.EngWhlTrqRatio_TCM = 0; // Implausible
    } else {
        eng_rq2_tcm.EngWhlTrqRatio_TCM = ratio * 100;
    }
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
uint8_t cvn_counter = 0;

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

void Egs53Can::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) {
    if(this->ecm_ecu.import_frames(data, id, timestamp)) {
    } else if (this->fscm_ecu.import_frames(data, id, timestamp)) {
    } else if (this->tslm_ecu.import_frames(data, id, timestamp)) {
    }
}