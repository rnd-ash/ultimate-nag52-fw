#include "egs52_can.h"
#include <pins.h>
#include <esp_log.h>

GS_218 gs218;
GS_338 gs338;
GS_418 gs418;

#define LOG_TAG "egs52-can"

#define KWP_RX_ID 0x07E1
#define KWP_TX_ID 0x07E9

Egs52Can::Egs52Can() : AbstractCanHandler()
{
    // Set CAN Frame data
    gs338.raw = 0xFFFF1FFF00FF0000;
    gs418.raw = 0x0000000000000000;
    gs218.raw = 0x0000000000000000;

    // Set one time parameters
    this->can_tx_ms = 20; // 20ms for EGS52
    this->bsECU = BS_ECU();
    this->ewmECU = EWM_ECU();

    gs218.set_GSP_OK(true);
    gs218.set_G_G(false);
    gs218.set_GS_218_FPC_AAD(GS_218_FPC_AAD::SPORT); // TODO Handle 4WD models
    gs218.set_KD(false);
    gs218.set_GET_OK(true);
    gs218.set_MKRIECH(0xFF);
    gs218.set_ERROR(0);

    gs418.set_CVT(false);
    gs418.set_FRONT(false);
    gs418.set_ALL_WHEEL(false);

    // Setup CANbus

    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, CAN_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    g_config.tx_queue_len = 4;
    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    can_driver_install(&g_config, &t_config, &f_config);
    can_start();
    ESP_LOGD(LOG_TAG, "CAN module started");
    this->diag_endpoint = new IsoTpServer(KWP_RX_ID, KWP_TX_ID, 8, 20);
}

Egs52Can::~Egs52Can()
{
    vTaskDelete(this->task_handler_tx);
    vTaskDelete(this->task_handler_rx);
}

void Egs52Can::__start_thread_tx(void *_this) {
    static_cast<Egs52Can*>(_this)->tx_loop();
}

void Egs52Can::__start_thread_rx(void *_this) {
    static_cast<Egs52Can*>(_this)->rx_loop();
}

void Egs52Can::start_tx_rx_loop() {
    if (this->task_handler_tx == nullptr) {
        ESP_LOGD(LOG_TAG, "Starting CAN Tx thread");
        xTaskCreate(this->__start_thread_tx, "CAN Tx", 2048, this, 5, this->task_handler_tx);
    }
    if (this->task_handler_tx == nullptr) {
        ESP_LOGD(LOG_TAG, "Starting CAN Rx thread");
        xTaskCreate(this->__start_thread_rx, "CAN Rx", 8192, this, 5, this->task_handler_rx);
    }
    //this->diag_endpoint->create_server_task();
}

void Egs52Can::rx_loop() {
    can_message_t rx;
    can_status_info_t can_status;
    while(true) {
        can_get_status_info(&can_status);
        while (can_status.msgs_to_rx > 0) {
            if (can_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK && rx.data_length_code != 0 && rx.flags == 0) {
                uint64_t now = esp_timer_get_time() / 1000;
                bool located = false;
                if (this->bsECU.import_can_frame(&rx, now)) { 
                    located = true;
                } else if (this->ewmECU.import_can_frame(&rx, now)) {
                    located = true;
                } else if (rx.identifier == KWP_RX_ID && rx.data_length_code == 8 && this->diag_endpoint != nullptr) {
                    this->diag_endpoint->push_frame(&rx);
                }
                vTaskDelay(1 / portTICK_RATE_MS);
            } else {
                vTaskDelay(5 / portTICK_PERIOD_MS); // TODO error in Rx
            }
        }
        if (can_status.msgs_to_rx == 0) {
            vTaskDelay(5);
        }
    }
    vTaskDelete(this->task_handler_rx);
}

void Egs52Can::tx_loop() {
    can_message_t tx;
    tx.flags = CAN_MSG_FLAG_NONE; // For Mercs
    while(true) {
        uint64_t now = esp_timer_get_time();
        gs338.export_frame(&tx.identifier, tx.data, &tx.data_length_code);
        can_transmit(&tx, pdMS_TO_TICKS(10));
        vTaskDelay(1);
        gs218.export_frame(&tx.identifier, tx.data, &tx.data_length_code);
        can_transmit(&tx, pdMS_TO_TICKS(10));
        vTaskDelay(1);
        gs418.export_frame(&tx.identifier, tx.data, &tx.data_length_code);
        can_transmit(&tx, pdMS_TO_TICKS(10));
        if (this->diag_endpoint != nullptr && this->diag_endpoint->pop_frame(&tx)) {
            can_transmit(&tx, pdMS_TO_TICKS(10));
        }
        uint64_t sleep_time = ((esp_timer_get_time() - now) / 1000);
        if (sleep_time > this->can_tx_ms) {
            sleep_time = 1;
        } else {
            sleep_time = this->can_tx_ms - sleep_time;
        }
        vTaskDelay(sleep_time / portTICK_RATE_MS); 
    }
    vTaskDelete(this->task_handler_tx);
}

uint16_t Egs52Can::get_engine_rpm() {
    return 0;
}

void Egs52Can::get_rr_rpm(WheelRotation *dest) {
    BS_208* bs208 = this->bsECU.get_bs208();
    if (bs208 == nullptr) {
        dest->rpm = UINT16_MAX;
        dest->dir = WheelDirection::SNV;
    } else {
        dest->rpm = bs208->get_DHR();
        switch(bs208->get_DRTGHR()) {
            case BS_208_DRTGHR::PASSIVE:
                dest->dir = WheelDirection::PASSIVE;
                break;
            case BS_208_DRTGHR::FORWARD:
                dest->dir = WheelDirection::FORWARD;
                break;
            case BS_208_DRTGHR::REVERSE:
                dest->dir = WheelDirection::REVERSE;
                break;
            case BS_208_DRTGHR::SNV:
                dest->dir = WheelDirection::SNV;
                break;
            default:
                break;
        }
    }
}

void Egs52Can::get_rl_rpm(WheelRotation *dest) {
    BS_208* bs208 = this->bsECU.get_bs208();
    if (bs208 == nullptr) {
        dest->rpm = UINT16_MAX;
        dest->dir = WheelDirection::SNV;
    } else {
        dest->rpm = bs208->get_DHL();
        switch(bs208->get_DRTGHL()) {
            case BS_208_DRTGHL::PASSIVE:
                dest->dir = WheelDirection::PASSIVE;
                break;
            case BS_208_DRTGHL::FORWARD:
                dest->dir = WheelDirection::FORWARD;
                break;
            case BS_208_DRTGHL::REVERSE:
                dest->dir = WheelDirection::REVERSE;
                break;
            case BS_208_DRTGHL::SNV:
                dest->dir = WheelDirection::SNV;
                break;
            default:
                break;
        }
    }
}

void Egs52Can::get_fr_rpm(WheelRotation *dest) {
    
}

void Egs52Can::get_fl_rpm(WheelRotation *dest) {
    
}

int16_t Egs52Can::get_steering_angle() {
    return 0;
}

int16_t Egs52Can::get_ambient_temp() {
    return 0;
}

int16_t Egs52Can::get_engine_temp() {
    return 0;
}

bool Egs52Can::is_profile_toggle_pressed() {
    EWM_230* ptr = this->ewmECU.get_ewm230();
    if (ptr == nullptr) {
        return false;
    } else {
        return ptr->get_FPT();
    }
}

Gear Egs52Can::get_abs_target_lower_gear() {
    return Gear::PASSIVE;
}

Gear Egs52Can::get_abs_target_upper_gear() {
    return Gear::PASSIVE;
}

bool Egs52Can::get_abs_request_downshift() {
    return false;
}

bool Egs52Can::get_abs_request_gear_forced() {
    return false;
}

uint16_t Egs52Can::get_engine_static_torque() {
    return 0;
}

uint16_t Egs52Can::get_engine_max_torque_dyno() {
    return 0;
}

uint16_t Egs52Can::get_engine_max_torque() {
    return 0;
}

uint16_t Egs52Can::get_engine_min_torque() {
    return 0;
}

ShifterPosition Egs52Can::get_shifter_position() {
    EWM_230* ptr = this->ewmECU.get_ewm230();
    if (ptr == nullptr) {
        return ShifterPosition::SNV;
    } else {
        switch (ptr->get_WHC()) {
            case EWM_230_WHC::D: /** Selector lever in position "D" */
                return ShifterPosition::D;
            case EWM_230_WHC::PLUS: /** Selector lever in position "+" */
                return ShifterPosition::PLUS;
            case EWM_230_WHC::MINUS: /** Selector lever in position "-" */
                return ShifterPosition::MINUS;
            case EWM_230_WHC::N: /** Selector lever in position "N" */
                return ShifterPosition::N;
            case EWM_230_WHC::R: /** Selector lever in position "R" */
                return ShifterPosition::R;
            case EWM_230_WHC::P: /** Selector lever in position "P" */
                return ShifterPosition::P;
            case EWM_230_WHC::N_ZW_D: /** Selector lever in intermediate position "N-D" */
                return ShifterPosition::N_D;
            case EWM_230_WHC::R_ZW_N: /** Selector lever in intermediate position "R-N" */
                return ShifterPosition::R_N;
            case EWM_230_WHC::P_ZW_R: /** Selector lever in intermediate position "P-R" */
                return ShifterPosition::P_R;
            case EWM_230_WHC::SNV: /** Selector lever position implausible */
            default:
                return ShifterPosition::SNV;
        }
    }
}

void Egs52Can::set_is_safe_start(bool can_start) {
    gs218.set_ALF(can_start);
}

void Egs52Can::set_atf_temp(uint16_t temp) {
    gs418.set_T_GET(temp-50);
}

void Egs52Can::set_drive_profile(DriveProfileDisplay p) {
    this->temp = p;
    switch (p) {
        case DriveProfileDisplay::Agility:
            gs418.set_GS_418_FPC(GS_418_FPC::A);
            break;
        case DriveProfileDisplay::Comfort:
            gs418.set_GS_418_FPC(GS_418_FPC::C);
            break;
        case DriveProfileDisplay::Manual:
            gs418.set_GS_418_FPC(GS_418_FPC::M);
            break;
        case DriveProfileDisplay::Standard:
            gs418.set_GS_418_FPC(GS_418_FPC::S);
            break;
        case DriveProfileDisplay::Winter:
            gs418.set_GS_418_FPC(GS_418_FPC::W);
            break;
        case DriveProfileDisplay::FAILURE:
            gs418.set_GS_418_FPC(GS_418_FPC::F);
            break;
        default:
            gs418.set_GS_418_FPC(GS_418_FPC::SNV);
            break;
    }
}

void Egs52Can::set_display_message(DisplayMessage m) {
    if (m == DisplayMessage::None) {
        // Set drive profile to clear message
        this->set_drive_profile(this->temp);
    }

    if (this->temp == DriveProfileDisplay::Agility) {
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone:
                gs418.set_GS_418_FPC(GS_418_FPC::A_MGFB_WT);
                break;
            case DisplayMessage::ApplyBrake:
                gs418.set_GS_418_FPC(GS_418_FPC::A_MGBB);
                break;
            case DisplayMessage::RequestGearAgain:
                gs418.set_GS_418_FPC(GS_418_FPC::A_MGGEA);
                break;
            case DisplayMessage::ShiftLeverToN:
                gs418.set_GS_418_FPC(GS_418_FPC::A_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                gs418.set_GS_418_FPC(GS_418_FPC::A_MGSNN);
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418_FPC(GS_418_FPC::A_MGW);
                break;
            default:
                break; // Upshift and downshift request ignore
        }
    } else if (this->temp == DriveProfileDisplay::Comfort) {
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone:
                gs418.set_GS_418_FPC(GS_418_FPC::C_MGFB_WT);
                break;
            case DisplayMessage::ApplyBrake:
                gs418.set_GS_418_FPC(GS_418_FPC::C_MGBB);
                break;
            case DisplayMessage::RequestGearAgain:
                gs418.set_GS_418_FPC(GS_418_FPC::C_MGGEA);
                break;
            case DisplayMessage::ShiftLeverToN:
                gs418.set_GS_418_FPC(GS_418_FPC::C_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                gs418.set_GS_418_FPC(GS_418_FPC::C_MGSNN);
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418_FPC(GS_418_FPC::C_MGW);
                break;
            default:
                break; // Upshift and downshift request ignore
        }
    } else if (this->temp == DriveProfileDisplay::Manual) {
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone:
            case DisplayMessage::ApplyBrake:
            case DisplayMessage::RequestGearAgain:
                break;
            case DisplayMessage::ShiftLeverToN:
                gs418.set_GS_418_FPC(GS_418_FPC::M_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418_FPC(GS_418_FPC::M_MGW);
                break;
            case DisplayMessage::Upshift:
                gs418.set_GS_418_FPC(GS_418_FPC::UP);
                break;
            case DisplayMessage::Downshift:
                gs418.set_GS_418_FPC(GS_418_FPC::DOWN);
                break;
            default:
                break; // Upshift and downshift request ignore
        }
    } else if (this->temp == DriveProfileDisplay::Standard) {
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone:
                gs418.set_GS_418_FPC(GS_418_FPC::S_MGFB_WT);
                break;
            case DisplayMessage::ApplyBrake:
                gs418.set_GS_418_FPC(GS_418_FPC::S_MGBB);
                break;
            case DisplayMessage::RequestGearAgain:
                gs418.set_GS_418_FPC(GS_418_FPC::S_MGGEA);
                break;
            case DisplayMessage::ShiftLeverToN:
                gs418.set_GS_418_FPC(GS_418_FPC::S_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                gs418.set_GS_418_FPC(GS_418_FPC::S_MGSNN);
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418_FPC(GS_418_FPC::S_MGW);
                break;
            default:
                break; // Upshift and downshift request ignore
        }
    } else if (this->temp == DriveProfileDisplay::Winter) { // Winter mode (Doesn't have as many warnings??)
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone: // Beep beep "Apply parking brake!"
                break;
            case DisplayMessage::ApplyBrake: // "Apply parking brake"
                break;
            case DisplayMessage::RequestGearAgain: // Requesting gear again
                break;
            case DisplayMessage::ShiftLeverToN: // Shift lever request to 'N'
                gs418.set_GS_418_FPC(GS_418_FPC::W_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart: // Shift lever request to 'N' to start engine
                break;
            case DisplayMessage::VisitWorkshop: // Fault with gearbox. Visit workshop
                gs418.set_GS_418_FPC(GS_418_FPC::W_MGW);
                break;
            default: // Signal not valid (SNV) / Other value
                break; // Upshift and downshift request ignore
        }
    } else if (this->temp == DriveProfileDisplay::FAILURE) { // Gearbox is broken..limp mode profile
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone:
                break;
            case DisplayMessage::ApplyBrake:
                break;
            case DisplayMessage::RequestGearAgain:
                break;
            case DisplayMessage::ShiftLeverToN:
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418_FPC(GS_418_FPC::F_MGW);
                break;
            default:
                break; // Upshift and downshift request ignore
        }
    }
    // Ignore SNV
}

void Egs52Can::set_target_gear(Gear g) {
    switch (g) {
        case Gear::P:
            gs218.set_GS_218_GZC(GS_218_GZC::P);
            gs418.set_GS_418_GZC(GS_418_GZC::P);
            break;
        case Gear::N:
            gs218.set_GS_218_GZC(GS_218_GZC::N);
            gs418.set_GS_418_GZC(GS_418_GZC::N);
            break;
        case Gear::R1:
            gs218.set_GS_218_GZC(GS_218_GZC::R);
            gs418.set_GS_418_GZC(GS_418_GZC::R);
            break;
        case Gear::R2:
            gs218.set_GS_218_GZC(GS_218_GZC::R2);
            gs418.set_GS_418_GZC(GS_418_GZC::R2);
            break;
        case Gear::D1:
            gs218.set_GS_218_GZC(GS_218_GZC::D1);
            gs418.set_GS_418_GZC(GS_418_GZC::D1);
            break;
        case Gear::D2:
            gs218.set_GS_218_GZC(GS_218_GZC::D2);
            gs418.set_GS_418_GZC(GS_418_GZC::D2);
            break;
        case Gear::D3:
            gs218.set_GS_218_GZC(GS_218_GZC::D3);
            gs418.set_GS_418_GZC(GS_418_GZC::D3);
            break;
        case Gear::D4:
            gs218.set_GS_218_GZC(GS_218_GZC::D4);
            gs418.set_GS_418_GZC(GS_418_GZC::D4);
            break;
        case Gear::D5:
            gs218.set_GS_218_GZC(GS_218_GZC::D5);
            gs418.set_GS_418_GZC(GS_418_GZC::D5);
            break;
        case Gear::D6:
            gs218.set_GS_218_GZC(GS_218_GZC::D6);
            gs418.set_GS_418_GZC(GS_418_GZC::D6);
            break;
        case Gear::D7:
            gs218.set_GS_218_GZC(GS_218_GZC::D7);
            gs418.set_GS_418_GZC(GS_418_GZC::D7);
            break;
        case Gear::CANCEL:
            gs218.set_GS_218_GZC(GS_218_GZC::CANCEL);
            gs418.set_GS_418_GZC(GS_418_GZC::CANCEL);
            break;
        default:
            break;
    }
}

void Egs52Can::set_actual_gear(Gear g) {
    switch (g) {
        case Gear::P:
            gs218.set_GS_218_GIC(GS_218_GIC::P);
            gs418.set_GS_418_GIC(GS_418_GIC::P);
            break;
        case Gear::N:
            gs218.set_GS_218_GIC(GS_218_GIC::N);
            gs418.set_GS_418_GIC(GS_418_GIC::N);
            break;
        case Gear::R1:
            gs218.set_GS_218_GIC(GS_218_GIC::R);
            gs418.set_GS_418_GIC(GS_418_GIC::R);
            break;
        case Gear::R2:
            gs218.set_GS_218_GIC(GS_218_GIC::R2);
            gs418.set_GS_418_GIC(GS_418_GIC::R2);
            break;
        case Gear::D1:
            gs218.set_GS_218_GIC(GS_218_GIC::D1);
            gs418.set_GS_418_GIC(GS_418_GIC::D1);
            break;
        case Gear::D2:
            gs218.set_GS_218_GIC(GS_218_GIC::D2);
            gs418.set_GS_418_GIC(GS_418_GIC::D2);
            break;
        case Gear::D3:
            gs218.set_GS_218_GIC(GS_218_GIC::D3);
            gs418.set_GS_418_GIC(GS_418_GIC::D3);
            break;
        case Gear::D4:
            gs218.set_GS_218_GIC(GS_218_GIC::D4);
            gs418.set_GS_418_GIC(GS_418_GIC::D4);
            break;
        case Gear::D5:
            gs218.set_GS_218_GIC(GS_218_GIC::D5);
            gs418.set_GS_418_GIC(GS_418_GIC::D5);
            break;
        case Gear::D6:
            gs218.set_GS_218_GIC(GS_218_GIC::D6);
            gs418.set_GS_418_GIC(GS_418_GIC::D6);
            break;
        case Gear::D7:
            gs218.set_GS_218_GIC(GS_218_GIC::D7);
            gs418.set_GS_418_GIC(GS_418_GIC::D7);
            break;
        default:
            break;
    }
}

void Egs52Can::set_turbine_rpm(uint16_t rpm) {
    gs338.set_NTURBINE(rpm);
}

void Egs52Can::set_torque_loss_nm(uint16_t loss) {
    gs418.set_M_VERL(loss);
}

void Egs52Can::set_display_speed_step(SpeedStep disp) {
    switch (disp) {
        case SpeedStep::BLANK:
            gs418.set_GS_418_FSC(GS_418_FSC::BLANK);
            break;
        case SpeedStep::ONE:
            gs418.set_GS_418_FSC(GS_418_FSC::ONE);
            break;
        case SpeedStep::TWO:
            gs418.set_GS_418_FSC(GS_418_FSC::TWO);
            break;
        case SpeedStep::THREE:
            gs418.set_GS_418_FSC(GS_418_FSC::THREE);
            break;
        case SpeedStep::FOUR:
            gs418.set_GS_418_FSC(GS_418_FSC::FOUR);
            break;
        case SpeedStep::FIVE:
            gs418.set_GS_418_FSC(GS_418_FSC::FUENF);
            break;
        case SpeedStep::SIX:
            gs418.set_GS_418_FSC(GS_418_FSC::SIX);
            break;
        case SpeedStep::SEVEN:
            gs418.set_GS_418_FSC(GS_418_FSC::SEVEN);
            break;
        case SpeedStep::A:
            gs418.set_GS_418_FSC(GS_418_FSC::A);
            break;
        case SpeedStep::D:
            gs418.set_GS_418_FSC(GS_418_FSC::D);
            break;
        case SpeedStep::R:
            gs418.set_GS_418_FSC(GS_418_FSC::R);
            break;
        case SpeedStep::F:
            gs418.set_GS_418_FSC(GS_418_FSC::F);
            break;
        case SpeedStep::N:
            gs418.set_GS_418_FSC(GS_418_FSC::N);
            break;
        case SpeedStep::P:
            gs418.set_GS_418_FSC(GS_418_FSC::P);
            break;
        default:
            break;
    }
}

void Egs52Can::set_status_error_check(ErrorCheck e) {
    switch (e) {
        case ErrorCheck::OK:
            gs218.set_GS_218_FEHLPRF_ST(GS_218_FEHLPRF_ST::OK);
            break;
        case ErrorCheck::WAIT:
            gs218.set_GS_218_FEHLPRF_ST(GS_218_FEHLPRF_ST::WAIT);
            break;
        case ErrorCheck::ERROR:
            gs218.set_GS_218_FEHLPRF_ST(GS_218_FEHLPRF_ST::ERROR);
            break;
        default:
            break;
    }
}

void Egs52Can::set_shifter_possition(ShifterPosition g) {
    switch (g) {
        case ShifterPosition::P:
            gs418.set_GS_418_WHST(GS_418_WHST::P);
            break;
        case ShifterPosition::P_R:
        case ShifterPosition::R:
        case ShifterPosition::R_N:
            gs418.set_GS_418_WHST(GS_418_WHST::R);
            break;
        case ShifterPosition::N:
            gs418.set_GS_418_WHST(GS_418_WHST::N);
            break;
        case ShifterPosition::N_D:
        case ShifterPosition::D:
        case ShifterPosition::MINUS:
        case ShifterPosition::PLUS:
            gs418.set_GS_418_WHST(GS_418_WHST::D);
            break;
        case ShifterPosition::SNV:
            gs418.set_GS_418_WHST(GS_418_WHST::SNV);
            break;
        default:
            break;
    }
}


Egs52Can* egs52_can_handler = nullptr;