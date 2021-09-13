#include "egs52_can.h"
#include <pins.h>

GS_218 gs218;
GS_338 gs338;
GS_418 gs418;

Egs52Can::Egs52Can() : AbstractCanHandler()
{
    // Set CAN Frame data
    gs338.raw = 0xFFFF1FFF00FF0000;
    gs418.raw = 0x0000000000000000;
    gs218.raw = 0x0000000000000000;

    // Set one time parameters
    this->can_tx_ms = 20; // 20ms for EGS52

    gs218.set_GSP_OK(true);
    gs218.set_G_G(false);
    gs218.set_GS_218H_FPC_AAD(0);
    gs218.set_KD(false);
    gs218.set_GET_OK(true);
    gs218.set_MKRIECH(0xFF);
    gs218.set_ERROR(0);

    gs418.set_CVT(false);
    gs418.set_FRONT(false);
    gs418.set_ALL_WHEEL(false);

    // Setup CANbus

    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, CAN_MODE_NORMAL);
    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    can_driver_install(&g_config, &t_config, &f_config);
    can_start();
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
        xTaskCreate(this->__start_thread_tx, "CAN Tx", 2048, this, 5, this->task_handler_tx);
    }
    if (this->task_handler_tx == nullptr) {
        xTaskCreate(this->__start_thread_rx, "CAN Rx", 8192, this, 5, this->task_handler_rx);
    }
}

void Egs52Can::rx_loop() {
    can_message_t rx;
    while(true) {
        if (can_receive(&rx, pdMS_TO_TICKS(10) == ESP_OK)) {
            uint64_t now = esp_timer_get_time() / 1000;
            if (this->bsECU.import_can_frame(&rx, now)) { 
                vTaskDelay(10 / portTICK_PERIOD_MS); // No data
            }
        } else {
            vTaskDelay(10 / portTICK_PERIOD_MS); // No data
        }
    }
    vTaskDelete(this->task_handler_rx);
}

void Egs52Can::tx_loop() {
    can_message_t tx;
    tx.flags = CAN_MSG_FLAG_NONE; // For Mercs
    bool positive = false;
    while(true) {
        uint64_t now = esp_timer_get_time();
        gs218.export_frame(&tx.identifier, tx.data, &tx.data_length_code);
        can_transmit(&tx, pdMS_TO_TICKS(10));
        gs338.export_frame(&tx.identifier, tx.data, &tx.data_length_code);
        can_transmit(&tx, pdMS_TO_TICKS(10));
        gs418.export_frame(&tx.identifier, tx.data, &tx.data_length_code);
        can_transmit(&tx, pdMS_TO_TICKS(10));
        uint64_t elapsed_ms = (esp_timer_get_time() - now) / 1000;
        vTaskDelay(elapsed_ms / portTICK_RATE_MS);
        positive = !positive;
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
            case BS_208H_DRTGHR::PASSIVE:
                dest->dir = WheelDirection::PASSIVE;
                break;
            case BS_208H_DRTGHR::FORWARD:
                dest->dir = WheelDirection::FORWARD;
                break;
            case BS_208H_DRTGHR::REVERSE:
                dest->dir = WheelDirection::REVERSE;
                break;
            case BS_208H_DRTGHR::SNV:
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
            case BS_208H_DRTGHL::PASSIVE:
                dest->dir = WheelDirection::PASSIVE;
                break;
            case BS_208H_DRTGHL::FORWARD:
                dest->dir = WheelDirection::FORWARD;
                break;
            case BS_208H_DRTGHL::REVERSE:
                dest->dir = WheelDirection::REVERSE;
                break;
            case BS_208H_DRTGHL::SNV:
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
    return false;
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

void Egs52Can::set_is_safe_start(bool can_start) {
    
}

void Egs52Can::set_atf_temp(uint16_t temp) {
    gs418.set_T_GET((uint8_t)temp);
}

void Egs52Can::set_drive_profile(DriveProfileDisplay p) {
    this->temp = p;
    switch (p) {
        case DriveProfileDisplay::Agility:
            gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::A);
            break;
        case DriveProfileDisplay::Comfort:
            gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::C);
            break;
        case DriveProfileDisplay::Manual:
            gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::M);
            break;
        case DriveProfileDisplay::Standard:
            gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::S);
            break;
        case DriveProfileDisplay::Winter:
            gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::W);
            break;
        case DriveProfileDisplay::FAILURE:
            gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::F);
            break;
        default:
            gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::SNV);
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
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::A_MGFB_WT);
                break;
            case DisplayMessage::ApplyBrake:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::A_MGBB);
                break;
            case DisplayMessage::RequestGearAgain:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::A_MGGEA);
                break;
            case DisplayMessage::ShiftLeverToN:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::A_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::A_MGSNN);
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::A_MGW);
                break;
            default:
                break; // Upshift and downshift request ignore
        }
    } else if (this->temp == DriveProfileDisplay::Comfort) {
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::C_MGFB_WT);
                break;
            case DisplayMessage::ApplyBrake:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::C_MGBB);
                break;
            case DisplayMessage::RequestGearAgain:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::C_MGGEA);
                break;
            case DisplayMessage::ShiftLeverToN:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::C_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::C_MGSNN);
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::C_MGW);
                break;
            default:
                break; // Upshift and downshift request ignore
        }
    } else if (this->temp == DriveProfileDisplay::Manual) {
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone:
                break;
            case DisplayMessage::ApplyBrake:
                break;
            case DisplayMessage::RequestGearAgain:
                break;
            case DisplayMessage::ShiftLeverToN:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::M_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::M_MGW);
                break;
            case DisplayMessage::Upshift:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::UP);
                break;
            case DisplayMessage::Downshift:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::DOWN);
                break;
            default:
                break; // Upshift and downshift request ignore
        }
    } else if (this->temp == DriveProfileDisplay::Standard) {
        switch (m) {
            case DisplayMessage::ActuateParkingBrake_WarningTone:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::S_MGFB_WT);
                break;
            case DisplayMessage::ApplyBrake:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::S_MGBB);
                break;
            case DisplayMessage::RequestGearAgain:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::S_MGGEA);
                break;
            case DisplayMessage::ShiftLeverToN:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::S_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::S_MGSNN);
                break;
            case DisplayMessage::VisitWorkshop:
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::S_MGW);
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
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::W_MGN);
                break;
            case DisplayMessage::ShiftLeverToNToStart: // Shift lever request to 'N' to start engine
                break;
            case DisplayMessage::VisitWorkshop: // Fault with gearbox. Visit workshop
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::W_MGW);
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
                gs418.set_GS_418H_FPC((uint8_t)GS_418H_FPC::F_MGW);
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
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::P);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::P);
            break;
        case Gear::N:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::N);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::N);
            break;
        case Gear::R1:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::R);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::R);
            break;
        case Gear::R2:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::R2);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::R2);
            break;
        case Gear::D1:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::D1);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::D1);
            break;
        case Gear::D2:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::D2);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::D2);
            break;
        case Gear::D3:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::D3);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::D3);
            break;
        case Gear::D4:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::D4);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::D4);
            break;
        case Gear::D5:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::D5);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::D5);
            break;
        case Gear::D6:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::D6);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::D6);
            break;
        case Gear::D7:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::D7);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::D7);
        case Gear::CANCEL:
            gs218.set_GS_218H_GZC((uint8_t)GS_218H_GZC::CANCEL);
            gs418.set_GS_418H_GZC((uint8_t)GS_418H_GZC::CANCEL);
            break;
        default:
            break;
    }
}

void Egs52Can::set_actual_gear(Gear g) {
    switch (g) {
        case Gear::P:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::P);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::P);
            break;
        case Gear::N:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::N);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::N);
            break;
        case Gear::R1:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::R);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::R);
            break;
        case Gear::R2:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::R2);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::R2);
            break;
        case Gear::D1:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::D1);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::D1);
            break;
        case Gear::D2:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::D2);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::D2);
            break;
        case Gear::D3:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::D3);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::D3);
            break;
        case Gear::D4:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::D4);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::D4);
            break;
        case Gear::D5:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::D5);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::D5);
            break;
        case Gear::D6:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::D6);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::D6);
            break;
        case Gear::D7:
            gs218.set_GS_218H_GIC((uint8_t)GS_218H_GIC::D7);
            gs418.set_GS_418H_GIC((uint8_t)GS_418H_GIC::D7);
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
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::BLANK);
            break;
        case SpeedStep::ONE:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::ONE);
            break;
        case SpeedStep::TWO:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::TWO);
            break;
        case SpeedStep::THREE:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::THREE);
            break;
        case SpeedStep::FOUR:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::FOUR);
            break;
        case SpeedStep::FIVE:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::FUENF);
            break;
        case SpeedStep::SIX:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::SIX);
            break;
        case SpeedStep::SEVEN:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::SEVEN);
            break;
        case SpeedStep::A:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::A);
            break;
        case SpeedStep::D:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::D);
            break;
        case SpeedStep::R:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::R);
            break;
        case SpeedStep::F:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::F);
            break;
        case SpeedStep::N:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::N);
            break;
        case SpeedStep::P:
            gs418.set_GS_418H_FSC((uint8_t)GS_418H_FSC::P);
            break;
        default:
            break;
    }
}

void Egs52Can::set_status_error_check(ErrorCheck e) {
    switch (e) {
        case ErrorCheck::OK:
            gs218.set_GS_218H_FEHLPRF_ST((uint8_t)GS_218H_FEHLPRF_ST::OK);
            break;
        case ErrorCheck::WAIT:
            gs218.set_GS_218H_FEHLPRF_ST((uint8_t)GS_218H_FEHLPRF_ST::WAIT);
            break;
        case ErrorCheck::ERROR:
            gs218.set_GS_218H_FEHLPRF_ST((uint8_t)GS_218H_FEHLPRF_ST::ERROR);
            break;
        default:
            break;
    }
}

void Egs52Can::set_shifter_possition(ShifterPosition g) {
    switch (g) {
        case ShifterPosition::P:
            gs418.set_GS_418H_WHST((uint8_t)GS_418H_WHST::P);
            break;
        case ShifterPosition::P_R:
        case ShifterPosition::R:
        case ShifterPosition::R_N:
            gs418.set_GS_418H_WHST((uint8_t)GS_418H_WHST::R);
            break;
        case ShifterPosition::N:
            gs418.set_GS_418H_WHST((uint8_t)GS_418H_WHST::N);
            break;
        case ShifterPosition::N_D:
        case ShifterPosition::D:
        case ShifterPosition::MINUS:
        case ShifterPosition::PLUS:
            gs418.set_GS_418H_WHST((uint8_t)GS_418H_WHST::D);
            break;
        case ShifterPosition::SNV:
            gs418.set_GS_418H_WHST((uint8_t)GS_418H_WHST::SNV);
            break;
        default:
            break;
    }
}


Egs52Can* egs52_can_handler = nullptr;