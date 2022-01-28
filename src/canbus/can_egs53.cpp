#include "can_egs53.h"
#include "driver/twai.h"
#include "pins.h"
#include "gearbox_config.h"

#ifdef EGS53_MODE

uint8_t crcTable[256]; // For CRC only

Egs53Can::Egs53Can(const char* name, uint8_t tx_time_ms)
    : AbstractCan(name, tx_time_ms)
{
    // Firstly try to init CAN
    ESP_LOGI("EGS52_CAN", "CAN constructor called");
    twai_general_config_t gen_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    gen_config.intr_flags = ESP_INTR_FLAG_IRAM; // Set TWAI interrupt to IRAM (Enabled in menuconfig)!
    gen_config.rx_queue_len = 10;
    gen_config.tx_queue_len = 6;
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t res;
    res = twai_driver_install(&gen_config, &timing_config, &filter_config);
    if (res != ESP_OK) {
        ESP_LOGE("EGS52_CAN", "TWAI_DRIVER_INSTALL FAILED!: %s", esp_err_to_name(res));
    }
    res = twai_start();
    if (res != ESP_OK) {
        ESP_LOGE("EGS52_CAN", "TWAI_START FAILED!: %s", esp_err_to_name(res));
    }

    // Create CRC table
    uint8_t _crc;
    for (int i = 0; i < 0x100; i++) {
            _crc = i;
            for (uint8_t bit = 0; bit < 8; bit++) {
                _crc = (_crc & 0x80) ? ((_crc << 1) ^ 0x1D) : (_crc << 1);
            }
            crcTable[i] = _crc;
    }

    // CAN is OK!
    // Set default values
    this->sbw_rs_tcm.set_SBW_MsgTxmtId(SBW_RS_TCM_SBW_MsgTxmtId::EGS52); // We are EGS53
    this->sbw_rs_tcm.set_TSL_Posn_Rq(SBW_RS_TCM_TSL_Posn_Rq::IDLE); // Idle request (No SBW on EGS53)
    this->sbw_rs_tcm.set_TxSelSensPosn(0); // No dialing sensor on EGS53
// Tell engine which Mech style we are
#ifdef LARGE_NAG
    this->eng_rq2_tcm.set_TxMechStyle(ENG_RQ2_TCM_TxMechStyle::LARGE);
#else
    this->eng_rq2_tcm.set_TxMechStyle(ENG_RQ2_TCM_TxMechStyle::SMALL);
#endif
    this->eng_rq2_tcm.set_TxStyle(ENG_RQ2_TCM_TxStyle::SAT); // Stepped automatic gearbox
    this->eng_rq2_tcm.set_TxShiftStyle(ENG_RQ2_TCM_TxShiftStyle::MS); // Mechanical shifting (With EWM module)
    this->can_init_ok = true;
}

bool Egs53Can::begin_tasks() {
    if (!this->can_init_ok) { // Cannot init tasks if CAN is dead!
        return false;
    }
    // Prevent starting again
    if (this->rx_task == nullptr) {
        ESP_LOGI("EGS53_CAN", "Starting CAN Rx task");
        if (xTaskCreate(this->start_rx_task_loop, "EGS53_CAN_RX", 8192, this, 5, this->rx_task) != pdPASS) {
            ESP_LOGE("EGS53_CAN", "CAN Rx task creation failed!");
            return false;
        }
    }
    if (this->tx_task == nullptr) {
        ESP_LOGI("EGS53_CAN", "Starting CAN Tx task");
        if (xTaskCreate(this->start_tx_task_loop, "EGS53_CAN_TX", 8192, this, 5, this->tx_task) != pdPASS) {
            ESP_LOGE("EGS53_CAN", "CAN Tx task creation failed!");
            return false;
        }
    }
    return true; // Ready!
}

Egs53Can::~Egs53Can()
{
    if (this->rx_task != nullptr) {
        vTaskDelete(this->rx_task);
    }
    if (this->tx_task != nullptr) {
        vTaskDelete(this->tx_task);
    }
    // Delete CAN
    if (this->can_init_ok) {
        twai_stop();
        twai_driver_uninstall();
    }
}

WheelData Egs53Can::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    WHL_STAT2 whl_stat;
    if (this->ecm_ecu.get_WHL_STAT2(now, expire_time_ms*1000, &whl_stat)) {
        WheelDirection dir;
        switch(whl_stat.get_WhlDir_FR_Stat()) {
            case WHL_STAT2_WhlDir_FR_Stat::VOID:
                dir = WheelDirection::Stationary;
                break;
            case WHL_STAT2_WhlDir_FR_Stat::FORWARD:
                dir = WheelDirection::Forward;
                break;
            case WHL_STAT2_WhlDir_FR_Stat::BACKWARD:
                dir = WheelDirection::Reverse;
                break;
            default:
                dir = WheelDirection::SignalNotAvaliable;
                break;
        }
        return WheelData {
            .double_rpm = whl_stat.get_WhlRPM_FR(),
            .current_dir = dir
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvaliable
        };
    }
}

WheelData Egs53Can::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    WHL_STAT2 whl_stat;
    if (this->ecm_ecu.get_WHL_STAT2(now, expire_time_ms*1000, &whl_stat)) {
        WheelDirection dir;
        switch(whl_stat.get_WhlDir_FL_Stat()) {
            case WHL_STAT2_WhlDir_FL_Stat::VOID:
                dir = WheelDirection::Stationary;
                break;
            case WHL_STAT2_WhlDir_FL_Stat::FORWARD:
                dir = WheelDirection::Forward;
                break;
            case WHL_STAT2_WhlDir_FL_Stat::BACKWARD:
                dir = WheelDirection::Reverse;
                break;
            default:
                dir = WheelDirection::SignalNotAvaliable;
                break;
        }
        return WheelData {
            .double_rpm = whl_stat.get_WhlRPM_FL(),
            .current_dir = dir
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvaliable
        };
    }
}

WheelData Egs53Can::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    WHL_STAT2 whl_stat;
    if (this->ecm_ecu.get_WHL_STAT2(now, expire_time_ms*1000, &whl_stat)) {
        WheelDirection dir;
        switch(whl_stat.get_WhlDir_RR_Stat()) {
            case WHL_STAT2_WhlDir_RR_Stat::VOID:
                dir = WheelDirection::Stationary;
                break;
            case WHL_STAT2_WhlDir_RR_Stat::FORWARD:
                dir = WheelDirection::Forward;
                break;
            case WHL_STAT2_WhlDir_RR_Stat::BACKWARD:
                dir = WheelDirection::Reverse;
                break;
            default:
                dir = WheelDirection::SignalNotAvaliable;
                break;
        }
        return WheelData {
            .double_rpm = whl_stat.get_WhlRPM_RR(),
            .current_dir = dir
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvaliable
        };
    }
}

WheelData Egs53Can::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    WHL_STAT2 whl_stat;
    if (this->ecm_ecu.get_WHL_STAT2(now, expire_time_ms*1000, &whl_stat)) {
        WheelDirection dir;
        switch(whl_stat.get_WhlDir_RL_Stat()) {
            case WHL_STAT2_WhlDir_RL_Stat::VOID:
                dir = WheelDirection::Stationary;
                break;
            case WHL_STAT2_WhlDir_RL_Stat::FORWARD:
                dir = WheelDirection::Forward;
                break;
            case WHL_STAT2_WhlDir_RL_Stat::BACKWARD:
                dir = WheelDirection::Reverse;
                break;
            default:
                dir = WheelDirection::SignalNotAvaliable;
                break;
        }
        return WheelData {
            .double_rpm = whl_stat.get_WhlRPM_RL(),
            .current_dir = dir
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvaliable
        };
    }
}

ShifterPosition Egs53Can::get_shifter_position_ewm(uint64_t now, uint64_t expire_time_ms) {
    SBW_RS_ISM tslm;
    if (this->tslm_ecu.get_SBW_RS_ISM(now, expire_time_ms*1000, &tslm)) {
        switch (tslm.get_TSL_Posn_ISM()) {
            case SBW_RS_ISM_TSL_Posn_ISM::D:
                return ShifterPosition::D;
            case SBW_RS_ISM_TSL_Posn_ISM::N:
                return ShifterPosition::N;
            case SBW_RS_ISM_TSL_Posn_ISM::R:
                return ShifterPosition::R;
            case SBW_RS_ISM_TSL_Posn_ISM::P:
                return ShifterPosition::P;
            case SBW_RS_ISM_TSL_Posn_ISM::PLUS:
                return ShifterPosition::PLUS;
            case SBW_RS_ISM_TSL_Posn_ISM::MINUS:
                return ShifterPosition::MINUS;
            case SBW_RS_ISM_TSL_Posn_ISM::N_ZW_D:
                return ShifterPosition::N_D;
            case SBW_RS_ISM_TSL_Posn_ISM::R_ZW_N:
                return ShifterPosition::R_N;
            case SBW_RS_ISM_TSL_Posn_ISM::P_ZW_R:
                return ShifterPosition::P_R;
            case SBW_RS_ISM_TSL_Posn_ISM::SNA:
            default:
                return ShifterPosition::SignalNotAvaliable;
        }
    }
    return ShifterPosition::SignalNotAvaliable;
}

EngineType Egs53Can::get_engine_type(uint64_t now, uint64_t expire_time_ms) {
    return EngineType::Unknown;
}

bool Egs53Can::get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs53Can::get_kickdown(uint64_t now, uint64_t expire_time_ms) { // TODO
    ENG_RS3_PT eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(now, expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.get_KickDnSw_Psd();
    }
    return false;
}

uint8_t Egs53Can::get_pedal_value(uint64_t now, uint64_t expire_time_ms) {
    ENG_RS3_PT eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(now, expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.get_AccelPdlPosn_Raw(); // Use RAW position, not 'modified' value from ECM!
    }
    return 0;
}

int Egs53Can::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    ENG_RS2_PT rs2_pt;
    if (this->ecm_ecu.get_ENG_RS2_PT(now, expire_time_ms*1000, &rs2_pt)) {
        return (rs2_pt.get_EngTrqStatic() / 4) - 500;
    }
    return 0;
}

int Egs53Can::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    ENG_RS2_PT rs2_pt;
    if (this->ecm_ecu.get_ENG_RS2_PT(now, expire_time_ms*1000, &rs2_pt)) {
        return (rs2_pt.get_EngTrqMaxETC() / 4) - 500;
    }
    return 0;
}

int Egs53Can::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    ENG_RS2_PT rs2_pt;
    if (this->ecm_ecu.get_ENG_RS2_PT(now, expire_time_ms*1000, &rs2_pt)) {
        return (rs2_pt.get_EngTrqMinTTC() / 4) - 500;
    }
    return 0;
}

PaddlePosition Egs53Can::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    return PaddlePosition::None;
}

int16_t Egs53Can::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    ECM_A1 ecm_a1;
    if (this->ecm_ecu.get_ECM_A1(now, expire_time_ms*1000, &ecm_a1)) {
        return (ecm_a1.get_EngCoolTemp() - 40);
    }
    return INT16_MAX; // UNDEFINED
}

int16_t Egs53Can::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) { // TODO
    ECM_A1 ecm_a1;
    if (this->ecm_ecu.get_ECM_A1(now, expire_time_ms*1000, &ecm_a1)) {
        return (ecm_a1.get_EngOilTemp() - 40);
    }
    return INT16_MAX; // UNDEFINED
}

uint16_t Egs53Can::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    ENG_RS3_PT eng_rs3;
    if (this->ecm_ecu.get_ENG_RS3_PT(now, expire_time_ms*1000, &eng_rs3)) {
        return eng_rs3.get_EngRPM();
    }
    return UINT16_MAX; // UNDEFINED
}

bool Egs53Can::get_is_starting(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs53Can::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    SBW_RS_ISM tslm;
    if (this->tslm_ecu.get_SBW_RS_ISM(now, expire_time_ms*10000, &tslm)) {
        return tslm.get_TxDrvProgSw_Psd_V3();
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
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::P);
            break;
        case GearboxGear::Reverse_First:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::R);
            break;
        case GearboxGear::Reverse_Second:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::R_2);
            break;
        case GearboxGear::Neutral:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::N);
            break;
        case GearboxGear::First:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::D1);
            break;
        case GearboxGear::Second:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::D2);
            break;
        case GearboxGear::Third:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::D3);
            break;
        case GearboxGear::Fourth:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::D4);
            break;
        case GearboxGear::Fifth:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::D5);
            break;
        case GearboxGear::SignalNotAvaliable:
        default:
            this->eng_rq2_tcm.set_Gr(ENG_RQ2_TCM_Gr::SNA);
            break;
    }
}

void Egs53Can::set_target_gear(GearboxGear target) {
    switch (target) {
        case GearboxGear::Park:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::P);
            break;
        case GearboxGear::Reverse_First:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::R);
            break;
        case GearboxGear::Reverse_Second:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::R_2);
            break;
        case GearboxGear::Neutral:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::N);
            break;
        case GearboxGear::First:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::D1);
            break;
        case GearboxGear::Second:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::D2);
            break;
        case GearboxGear::Third:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::D3);
            break;
        case GearboxGear::Fourth:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::D4);
            break;
        case GearboxGear::Fifth:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::D5);
            break;
        case GearboxGear::SignalNotAvaliable:
        default:
            this->eng_rq2_tcm.set_Gr_Target(ENG_RQ2_TCM_Gr_Target::SNA);
            break;
    }
}

void Egs53Can::set_safe_start(bool can_start) {
    this->sbw_rs_tcm.set_StartLkSw(can_start);
    this->eng_rq1_tcm.set_EngSt_Enbl_Rq_TCM(can_start);
}

void Egs53Can::set_gearbox_temperature(uint16_t temp) {
    this->tcm_a1.set_TxOilTemp(temp + 50);
}

void Egs53Can::set_input_shaft_speed(uint16_t rpm) {
    this->tcm_a2.set_TxTurbineRPM(rpm);
}

void Egs53Can::set_is_all_wheel_drive(bool is_4wd) {
    
}

void Egs53Can::set_wheel_torque(uint16_t t) {

}

void Egs53Can::set_shifter_position(ShifterPosition pos) {
    switch (pos) {
        case ShifterPosition::P:
            this->sbw_rs_tcm.set_TxSelVlvPosn(SBW_RS_TCM_TxSelVlvPosn::P);
            this->tcm_a1.set_TSL_Posn_TCM(TCM_A1_TSL_Posn_TCM::P);
            break;
        case ShifterPosition::P_R:
            this->sbw_rs_tcm.set_TxSelVlvPosn(SBW_RS_TCM_TxSelVlvPosn::P_ZW_R);
            this->tcm_a1.set_TSL_Posn_TCM(TCM_A1_TSL_Posn_TCM::P);
            break;
        case ShifterPosition::R:
            this->sbw_rs_tcm.set_TxSelVlvPosn(SBW_RS_TCM_TxSelVlvPosn::R);
            this->tcm_a1.set_TSL_Posn_TCM(TCM_A1_TSL_Posn_TCM::R);
            break;
        case ShifterPosition::R_N:
            this->sbw_rs_tcm.set_TxSelVlvPosn(SBW_RS_TCM_TxSelVlvPosn::R_ZW_N);
            this->tcm_a1.set_TSL_Posn_TCM(TCM_A1_TSL_Posn_TCM::R);
            break;
        case ShifterPosition::N:
            this->sbw_rs_tcm.set_TxSelVlvPosn(SBW_RS_TCM_TxSelVlvPosn::N);
            this->tcm_a1.set_TSL_Posn_TCM(TCM_A1_TSL_Posn_TCM::N);
            break;
        case ShifterPosition::N_D:
            this->sbw_rs_tcm.set_TxSelVlvPosn(SBW_RS_TCM_TxSelVlvPosn::N_ZW_D);
            this->tcm_a1.set_TSL_Posn_TCM(TCM_A1_TSL_Posn_TCM::N);
            break;
        case ShifterPosition::D:
        case ShifterPosition::PLUS:
        case ShifterPosition::MINUS:
            this->sbw_rs_tcm.set_TxSelVlvPosn(SBW_RS_TCM_TxSelVlvPosn::D);
            this->tcm_a1.set_TSL_Posn_TCM(TCM_A1_TSL_Posn_TCM::D);
            break;
        case ShifterPosition::SignalNotAvaliable:
        default:
            this->sbw_rs_tcm.set_TxSelVlvPosn(SBW_RS_TCM_TxSelVlvPosn::SNA);
            this->tcm_a1.set_TSL_Posn_TCM(TCM_A1_TSL_Posn_TCM::SNA);
            break;
    }
}

void Egs53Can::set_gearbox_ok(bool is_ok) {
    this->tcm_a1.set_TCM_LHOM(!is_ok);
    this->tcm_a1.set_BasShftProg_Ok(is_ok);
}

void Egs53Can::set_torque_request(TorqueRequest request) {
    
}

void Egs53Can::set_requested_torque(uint16_t torque_nm) {
    
}

void Egs53Can::set_error_check_status(SystemStatusCheck ssc) {
    
}


void Egs53Can::set_turbine_torque_loss(uint16_t loss_nm) {
    
}

uint8_t x = 0;
unsigned long last_time = 0;
void Egs53Can::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
    switch (g) {
        case GearboxDisplayGear::One:
            manual_mode ? this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::M1)
            : this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::D1);
            break;
        case GearboxDisplayGear::Two:
            manual_mode ? this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::M2)
            : this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::D2);
            break;
        case GearboxDisplayGear::Three:
            manual_mode ? this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::M3)
            : this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::D3);
            break;
        case GearboxDisplayGear::Four:
            manual_mode ? this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::M4)
            : this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::D4);
            break;
        case GearboxDisplayGear::Five:
            manual_mode ? this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::M1)
            : this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::D1);
            break;
        case GearboxDisplayGear::P:
            this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::P);
            break;
        case GearboxDisplayGear::D:
            this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::D);
            break;
        case GearboxDisplayGear::N:
            this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::N);
            break;
        case GearboxDisplayGear::R:
            this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::R);
            break;
        case GearboxDisplayGear::Failure:
            this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::F);
            break;
        case GearboxDisplayGear::SNA:
        default:
            this->tcm_disp_rq.set_TxDrvPosn_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvPosn_Disp_Rq_TCM::BLANK);
            break;
    }
}

void Egs53Can::set_drive_profile(GearboxProfile p) {
    switch(p) {
        case GearboxProfile::Agility:
            this->tcm_disp_rq.set_TxDrvProg_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM::A);
            break;
        case GearboxProfile::Comfort:
            this->tcm_disp_rq.set_TxDrvProg_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM::C);
            break;
        case GearboxProfile::Standard:
            this->tcm_disp_rq.set_TxDrvProg_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM::S);
            break;
        case GearboxProfile::Winter:
            this->tcm_disp_rq.set_TxDrvProg_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM::W);
            break;
        case GearboxProfile::Manual:
            this->tcm_disp_rq.set_TxDrvProg_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM::M);
            break;
        case GearboxProfile::Failure:
            this->tcm_disp_rq.set_TxDrvProg_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM::F);
            break;
        default:
            this->tcm_disp_rq.set_TxDrvProg_Disp_Rq_TCM(TCM_DISP_RQ_TxDrvProg_Disp_Rq_TCM::BLANK);
            break;
    }
}

void Egs53Can::set_last_shift_time(uint16_t time_ms) {

}

void Egs53Can::set_race_start(bool race_start) {

}

void Egs53Can::set_solenoid_pwm(uint8_t duty, SolenoidName s) {

}

void Egs53Can::set_display_msg(GearboxMessage msg) {
    
}

/**
 * Calculates CRC based on SAE-J180.
 * As all the CAN Frames work the same, we always assume its the first 7 bytes, and result is to be placed
 * in the 8th byte in the frame
 */
void calc_crc_in_place(uint8_t* buffer) {
    // assume len = 7
    unsigned long crc;
    int i,bit;

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
[[noreturn]]
void Egs53Can::tx_task_loop() {
    uint64_t start_time;
    uint32_t taken;
    twai_message_t tx;
    tx.data_length_code = 8; // Always

    TCM_A1 tcm_a1_tx = {0};
    TCM_A2 tcm_a2_tx = {0};
    ENG_RQ1_TCM eng_rq1_tcm_tx = {0};
    ENG_RQ2_TCM eng_rq2_tcm_tx = {0};
    ENG_RQ3_TCM eng_rq3_tcm_tx = {0};
    SBW_RS_TCM sbw_rs_tcm_tx = {0};
    TCM_DISP_RQ tcm_disp_rq_tx = {0};
    NM_TCM nm_tcm_tx = {0};
    uint8_t counter = 0;
    uint8_t cvn_counter = 0;
    while(true) {
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
        // Now send CAN Data!

        start_time = esp_timer_get_time() / 1000;

        tcm_a2_tx.set_TCM_CALID_CVN_ErrNum(cvn_counter);
        cvn_counter++;
        if (cvn_counter == 0x14) {
            cvn_counter = 0;
        }

        // Set message counters
        eng_rq1_tcm_tx.set_MC_ENG_RQ1_TCM(msg_counter);
        eng_rq2_tcm_tx.set_MC_ENG_RQ2_TCM(msg_counter);
        eng_rq3_tcm_tx.set_MC_ENG_RQ3_TCM(msg_counter);
        sbw_rs_tcm_tx.set_MC_SBW_RS_TCM(msg_counter);
        msg_counter++; // Global for all messages out of TCM

        tx.identifier = ENG_RQ1_TCM_CAN_ID;
        to_bytes(eng_rq1_tcm_tx.raw, tx.data);
        calc_crc_in_place(tx.data);
        if (this->send_messages) { twai_transmit(&tx, 5); }
        
        tx.identifier = ENG_RQ2_TCM_CAN_ID;
        to_bytes(eng_rq2_tcm_tx.raw, tx.data);  
        calc_crc_in_place(tx.data);
        if (this->send_messages) { twai_transmit(&tx, 5); }

        tx.identifier = TCM_A2_CAN_ID;
        to_bytes(tcm_a2_tx.raw, tx.data);
        calc_crc_in_place(tx.data);
        if (this->send_messages) { twai_transmit(&tx, 5); }

        tx.identifier = TCM_A1_CAN_ID;
        to_bytes(tcm_a1_tx.raw, tx.data);
        if (this->send_messages) { twai_transmit(&tx, 5); }

        if (counter == 5) {
            tx.identifier = TCM_DISP_RQ_CAN_ID;
            to_bytes(tcm_disp_rq_tx.raw, tx.data);
            if (this->send_messages) { twai_transmit(&tx, 5); }
            counter = 0;
        }
        counter++;

        tx.identifier = SBW_RS_TCM_CAN_ID;
        to_bytes(sbw_rs_tcm_tx.raw, tx.data);
        calc_crc_in_place(tx.data);
        if (this->send_messages) { twai_transmit(&tx, 5); }

        // Todo handle additional ISOTP communication
        if (this->diag_tx_queue != nullptr) {
            DiagCanMessage buffer;
            if (xQueueReceive(*this->diag_tx_queue, (void*)(buffer), 0) == pdTRUE) {
                // Popped message!
                tx.data_length_code = 8;
                tx.identifier = this->diag_tx_id;
                memcpy(tx.data, buffer, 8);
                ESP_LOG_BUFFER_HEX_LEVEL("CAN_TX_DIAG", tx.data, 8, esp_log_level_t::ESP_LOG_INFO);
                twai_transmit(&tx, 5);
            }
        }
        taken = (esp_timer_get_time() / 1000) - start_time;
        if (taken < this->tx_time_ms) {
            vTaskDelay(this->tx_time_ms-taken / portTICK_PERIOD_MS);
        }
    }
}

[[noreturn]]
void Egs53Can::rx_task_loop() {
    twai_message_t rx;
    twai_status_info_t can_status;
    uint64_t now;
    uint64_t tmp;
    uint8_t i;
    while(true) {
        twai_get_status_info(&can_status);
        uint8_t f_count  = can_status.msgs_to_rx;
        if (f_count == 0) {
            vTaskDelay(4 / portTICK_PERIOD_MS); // Wait for buffer to have at least 1 frame
        } else { // We have frames, read them
            now = esp_timer_get_time()/1000;
            for(uint8_t x = 0; x < f_count; x++) { // Read all frames
                if (twai_receive(&rx, pdMS_TO_TICKS(2)) == ESP_OK && rx.data_length_code != 0 && rx.flags == 0) {
                    tmp = 0;
                    for(i = 0; i < rx.data_length_code; i++) {
                        tmp |= (uint64_t)rx.data[i] << (8*(7-i));
                    }
                    if(this->ecm_ecu.import_frames(tmp, rx.identifier, now)) {
                    } else if (this->fscm_ecu.import_frames(tmp, rx.identifier, now)) {
                    } else if (this->tslm_ecu.import_frames(tmp, rx.identifier, now)) {
                    } else if (this->diag_rx_id != 0 && rx.identifier == this->diag_rx_id) {
                        // ISO-TP Diag endpoint
                        if (this->diag_rx_queue != nullptr && rx.data_length_code == 8) {
                            // Send the frame
                            DiagCanMessage msg;
                            memcpy(msg, rx.data, 8);
                            if (xQueueSend(*this->diag_rx_queue, msg, 0) != pdTRUE) {
                                ESP_LOGE("EGS53_CAN","Discarded ISO-TP endpoint frame. Queue send failed");
                            }
                        }
                    } 
                } else {
                    vTaskDelay(2 / portTICK_PERIOD_MS);
                }
            }
            vTaskDelay(1 / portTICK_PERIOD_MS); // Reset watchdog here
        }
    }
}

#endif