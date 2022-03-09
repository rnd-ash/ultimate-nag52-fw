#include "can_egs52.h"
#include "driver/twai.h"
#include "pins.h"
#include "gearbox_config.h"

#ifdef EGS52_MODE

Egs52Can::Egs52Can(const char* name, uint8_t tx_time_ms)
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
    // CAN is OK!

    // Set default values
    this->set_target_gear(GearboxGear::SignalNotAvaliable);
    this->set_actual_gear(GearboxGear::SignalNotAvaliable);
    this->set_shifter_position(ShifterPosition::SignalNotAvaliable);
    this->gs218.set_GIC(GS_218h_GIC::G_SNV);
    gs218.set_CALID_CVN_AKT(true);
    gs218.set_G_G(true);
    // Set profile to N/A for now
    this->set_drive_profile(GearboxProfile::Underscore);
    // Set no message
    this->set_display_msg(GearboxMessage::None);
    this->gs218.set_SCHALT(true);
    this->gs218.set_MKRIECH(0xFF);

// Set permanent configuration frame
#ifdef FOUR_MATIC
    this->gs418.set_ALLRAD(true);
#else
    this->gs418.set_ALLRAD(false);
#endif
    this->gs418.set_FRONT(false); // Primary rear wheel drive
    this->gs418.set_CVT(false); // Not CVT gearbox
    this->gs418.set_MECH(GS_418h_MECH::GROSS); // Small 722.6 for now! (TODO Handle 580)


    // Covers setting NAB, a couple unknown but static values,
    // and Input RPM to 0 
    gs338.raw = 0xFFFF1FFF00FF0000;
    this->can_init_ok = true;
}

bool Egs52Can::begin_tasks() {
    if (!this->can_init_ok) { // Cannot init tasks if CAN is dead!
        return false;
    }
    // Prevent starting again
    if (this->rx_task == nullptr) {
        ESP_LOGI("EGS52_CAN", "Starting CAN Rx task");
        if (xTaskCreate(this->start_rx_task_loop, "EGS52_CAN_RX", 8192, this, 5, this->rx_task) != pdPASS) {
            ESP_LOGE("EGS52_CAN", "CAN Rx task creation failed!");
            return false;
        }
    }
    if (this->tx_task == nullptr) {
        ESP_LOGI("EGS52_CAN", "Starting CAN Tx task");
        if (xTaskCreate(this->start_tx_task_loop, "EGS52_CAN_TX", 8192, this, 5, this->tx_task) != pdPASS) {
            ESP_LOGE("EGS52_CAN", "CAN Tx task creation failed!");
            return false;
        }
    }
    return true; // Ready!
}

Egs52Can::~Egs52Can()
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

WheelData Egs52Can::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData Egs52Can::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData Egs52Can::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_200 bs200;
    if (this->esp_ecu.get_BS_200(now, expire_time_ms*1000, &bs200)) {
        WheelDirection d = WheelDirection::SignalNotAvaliable;
        switch(bs200.get_DRTGVR()) {
            case BS_200h_DRTGVR::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_200h_DRTGVR::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_200h_DRTGVR::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_200h_DRTGVR::SNV:
            default:
                break;
        }

        return WheelData {
            .double_rpm = bs200.get_DVR(),
            .current_dir = d
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvaliable
        };
    }
}

WheelData Egs52Can::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_200 bs200;
    if (this->esp_ecu.get_BS_200(now, expire_time_ms*1000, &bs200)) {
        WheelDirection d = WheelDirection::SignalNotAvaliable;
        switch(bs200.get_DRTGVL()) {
            case BS_200h_DRTGVL::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_200h_DRTGVL::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_200h_DRTGVL::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_200h_DRTGVL::SNV:
            default:
                break;
        }

        return WheelData {
            .double_rpm = bs200.get_DVL(),
            .current_dir = d
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvaliable
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
                return ShifterPosition::SignalNotAvaliable;
        }
    } else {
        return ShifterPosition::SignalNotAvaliable;
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

PaddlePosition Egs52Can::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    SBW_232 sbw;
    if (misc_ecu.get_SBW_232(now, expire_time_ms, &sbw)) { // 50ms timeout
        switch (sbw.get_LRT_PM3()) {
            case SBW_232h_LRT_PM3::PLUS:
                return PaddlePosition::Plus;
            case SBW_232h_LRT_PM3::MINUS:
                return PaddlePosition::Minus;
            default:
                return PaddlePosition::None;
        }
    } else {
        return PaddlePosition::None;
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

bool Egs52Can::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    EWM_230 ewm230;
    if (this->ewm_ecu.get_EWM_230(now, expire_time_ms, &ewm230)) {
        return ewm230.get_FPT();
    } else {
        return false;
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
        case GearboxGear::SignalNotAvaliable:
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
        case GearboxGear::SignalNotAvaliable:
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
        case ShifterPosition::SignalNotAvaliable:
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

void Egs52Can::set_torque_request(TorqueRequest request) {
    switch(request) {
        case TorqueRequest::Maximum:
            gs218.set_MMIN_EGS(false);
            gs218.set_MMAX_EGS(true);
            break;
        case TorqueRequest::Minimum:
            gs218.set_MMIN_EGS(true);
            gs218.set_MMAX_EGS(false);
            break;
        case TorqueRequest::None:
        default:
            gs218.set_MMIN_EGS(false);
            gs218.set_MMAX_EGS(false);
            break;
    }
}

void Egs52Can::set_requested_torque(uint16_t torque_nm) {
    gs218.set_M_EGS((torque_nm + 500) * 4);
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
            gs418.set_FPC(GS_418h_FPC::A);
            break;
        case GearboxProfile::Comfort:
            gs418.set_FPC(GS_418h_FPC::C);
            break;
        case GearboxProfile::Winter:
            gs418.set_FPC(GS_418h_FPC::W);
            break;
        case GearboxProfile::Failure:
            gs418.set_FPC(GS_418h_FPC::F);
            break;
        case GearboxProfile::Standard:
            gs418.set_FPC(GS_418h_FPC::S);
            break;
        case GearboxProfile::Manual:
            gs418.set_FPC(GS_418h_FPC::M);
            break;
        case GearboxProfile::Underscore:
            gs418.set_FPC(GS_418h_FPC::_);
            break;
        default:
            break;
    }
    // Update display message as well
    this->set_display_msg(this->curr_message);
}

void Egs52Can::set_solenoid_pwm(uint8_t duty, SolenoidName s) {
    switch (s) {
        case SolenoidName::Y3:
            gs558.set_y3_pwm(duty);
            break;
        case SolenoidName::Y4:
            gs558.set_y4_pwm(duty);
            break;
        case SolenoidName::Y5:
            gs558.set_y5_pwm(duty);
            break;
        case SolenoidName::SPC:
            gs558.set_spc_pwm(duty);
            break;
        case SolenoidName::MPC:
            gs558.set_mpc_pwm(duty);
            break;
        case SolenoidName::TCC:
            gs558.set_tcc_pwm(duty);
            break;
        default:
            break;
    }
}

void Egs52Can::set_display_msg(GearboxMessage msg) {
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
}

void Egs52Can::set_last_shift_time(uint16_t time_ms) {
    gs558.set_shift_time(time_ms);
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

inline void to_bytes(uint64_t src, uint8_t* dst) {
    for(uint8_t i = 0; i < 8; i++) {
        dst[7-i] = src & 0xFF;
        src >>= 8;
    }
}

[[noreturn]]
void Egs52Can::tx_task_loop() {
    uint64_t start_time;
    uint32_t taken;
    twai_message_t tx;
    tx.data_length_code = 8; // Always
    GS_338 gs_338tx;
    GS_218 gs_218tx;
    GS_418 gs_418tx;
    GS_558_CUSTOM gs_558tx;
    uint8_t cvn_counter = 0;
    bool toggle = false;
    bool time_to_toggle = false;
    while(true) {
        // Copy current CAN frame values to here so we don't
        // accidentally modify parity calculations
        gs_338tx = {gs338.raw};
        gs_218tx = {gs218.raw};
        gs_418tx = {gs418.raw};
        gs_558tx = {gs558.raw};

        // Firstly we have to deal with toggled bits!
        // As toggle bits need to be toggled every 40ms,
        // and egs52 Tx interval is 20ms,
        // we can achieve this with 2 booleans
        gs_218tx.set_MTGL_EGS(toggle);
        gs_418tx.set_FMRADTGL(toggle);
        // Now do parity calculations
        gs_218tx.set_MPAR_EGS(calc_torque_parity(gs_218tx.raw >> 48));
        gs_418tx.set_FMRADPAR(calc_torque_parity(gs_418tx.raw & 0xFFFF));
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
        start_time = esp_timer_get_time() / 1000;
        tx.identifier = GS_338_CAN_ID;
        to_bytes(gs_338tx.raw, tx.data);
        if (this->send_messages) { twai_transmit(&tx, 5); }
        tx.identifier = GS_218_CAN_ID;
        to_bytes(gs_218tx.raw, tx.data);
        if (this->send_messages) { twai_transmit(&tx, 5); }
        tx.identifier = GS_418_CAN_ID;
        to_bytes(gs_418tx.raw, tx.data);
        if (this->send_messages) { twai_transmit(&tx, 5); }
        tx.identifier = GS_CUSTOM_558_CAN_ID;
        to_bytes(gs_558tx.raw, tx.data);
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
void Egs52Can::rx_task_loop() {
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
                    if(this->ecu_ms.import_frames(tmp, rx.identifier, now)) {
                    } else if (this->esp_ecu.import_frames(tmp, rx.identifier, now)) {
                    } else if (this->ewm_ecu.import_frames(tmp, rx.identifier, now)) {
                    } else if (this->misc_ecu.import_frames(tmp, rx.identifier, now)) {
                    } else if (this->diag_rx_id != 0 && rx.identifier == this->diag_rx_id) {
                        // ISO-TP Diag endpoint
                        if (this->diag_rx_queue != nullptr && rx.data_length_code == 8) {
                            // Send the frame
                            DiagCanMessage msg;
                            memcpy(msg, rx.data, 8);
                            if (xQueueSend(*this->diag_rx_queue, msg, 0) != pdTRUE) {
                                ESP_LOGE("EGS52_CAN","Discarded ISO-TP endpoint frame. Queue send failed");
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