#include "can_egs52.h"
#include "driver/twai.h"
#include "pins.h"

Egs52Can::Egs52Can(const char* name, uint8_t tx_time_ms)
    : AbstractCan(name, tx_time_ms)
{
    // Firstly try to init CAN
    ESP_LOGI("EGS52_CAN", "CAN constructor called");
    twai_general_config_t gen_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
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
    this->gs218.set_GZC(GS_218h_GZC::G_SNV);
    this->gs218.set_GIC(GS_218h_GIC::G_SNV);
    gs218.set_CALID_CVN_AKT(true);

    // Convers setting NAB, a couple unknown but static values,
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

WheelData Egs52Can::get_front_right_wheel() {
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData Egs52Can::get_front_left_wheel() {
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData Egs52Can::get_rear_right_wheel() {
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData Egs52Can::get_rear_left_wheel() {
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

ShifterPosition Egs52Can::get_shifter_position_ewm() {
    EWM_230 dest;
    if (this->ewm_ecu.get_EWM_230(esp_timer_get_time(), 1000 * 100, &dest)) {
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

EngineType Egs52Can::get_engine_type() {
    MS_608 ms608;
    // This signal can be valid for up to 1000ms
    if (this->ecu_ms.get_MS_608(esp_timer_get_time(), 1000*1000, &ms608)) {
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

bool Egs52Can::get_engine_is_limp() {
    return false;
}

bool Egs52Can::get_kickdown() {
    return false;
}

uint8_t Egs52Can::get_pedal_value() {
    return 0;
}

uint16_t Egs52Can::get_static_engine_torque() {
    return 0;
}

uint16_t Egs52Can::get_maximum_engine_torque() {
    return 0;
}

uint16_t Egs52Can::get_minimum_engine_torque() {
    return 0;
}

PaddlePosition Egs52Can::get_paddle_position() {
    return PaddlePosition::None;
}

uint16_t Egs52Can::get_engine_coolant_temp() {
    return 0;
}

uint16_t Egs52Can::get_engine_oil_temp() {
    return 0;
}

uint16_t Egs52Can::get_engine_rpm() {
    return 0;
}

bool Egs52Can::get_is_starting() {
    return false;
}

void Egs52Can::set_clutch_status(ClutchStatus status) {

}

void Egs52Can::set_actual_gear(GearboxGear actual) {

}

void Egs52Can::set_target_gear(GearboxGear target) {

}

void Egs52Can::set_safe_start(bool can_start) {

}

void Egs52Can::set_gearbox_temperature(uint16_t temp) {

}

void Egs52Can::set_input_shaft_speed(uint16_t rpm) {
    gs338.set_NTURBINE(rpm);
}

void Egs52Can::set_is_all_wheel_drive(bool is_4wd) {

}

void Egs52Can::set_wheel_torque(uint16_t t) {

}

void Egs52Can::set_shifter_position(ShifterPosition pos) {
    switch (pos) {
        case ShifterPosition::P:
            gs418.set_WHST(GS_418h_WHST::P);
            break;
        case ShifterPosition::P_R:
        case ShifterPosition::R:
            gs418.set_WHST(GS_418h_WHST::R);
            break;
        case ShifterPosition::R_N:
        case ShifterPosition::N:
            gs418.set_WHST(GS_418h_WHST::N);
            break;
        case ShifterPosition::N_D:
        case ShifterPosition::D:
        case ShifterPosition::PLUS:
        case ShifterPosition::MINUS:
            gs418.set_WHST(GS_418h_WHST::D);
            break;
        case ShifterPosition::SignalNotAvaliable:
        default: 
            gs418.set_WHST(GS_418h_WHST::SNV);
            break;
    }
}

void Egs52Can::set_gearbox_ok(bool is_ok) {
    gs218.set_GET_OK(is_ok); // Gearbox OK
    gs218.set_GSP_OK(is_ok); // Gearbox profile OK
    gs218.set_GS_NOTL(!is_ok); // Emergency mode activated
}

void Egs52Can::set_torque_request(TorqueRequest request) {

}

void Egs52Can::set_requested_torque(uint16_t torque_nm) {

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
    uint8_t cvn_counter = 0;
    bool toggle = false;
    bool time_to_toggle = false;
    while(true) {
        // Copy current CAN frame values to here so we don't
        // accidentally modify parity calculations
        gs_338tx = {gs338.raw};
        gs_218tx = {gs218.raw};
        gs_418tx = {gs418.raw};
        // Firstly we have to deal with toggled parity bits!
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
        gs_218tx.set_FEHLER(cvn_counter);
        time_to_toggle = !time_to_toggle;
        cvn_counter++;

        // Now send CAN Data!

        /**
         * TX order of EGS52:
         * GS_338
         * GS_218
         * GS_418
         */
        start_time = esp_timer_get_time() / 1000;
        tx.identifier = GS_338_CAN_ID;
        to_bytes(gs_338tx.raw, tx.data);
        twai_transmit(&tx, 5);
        tx.identifier = GS_218_CAN_ID;
        to_bytes(gs_218tx.raw, tx.data);
        twai_transmit(&tx, 5);
        tx.identifier = GS_418_CAN_ID;
        to_bytes(gs_418tx.raw, tx.data);
        twai_transmit(&tx, 5);
        // Todo handle additional ISOTP communication
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
        bool have_frames  = false;
        while(can_status.msgs_to_rx > 0) {
            have_frames = true;
            if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK && rx.data_length_code != 0 && rx.flags == 0) {
                now = esp_timer_get_time();
                tmp = 0;

                for(i = 0; i < 8; i++) {
                    tmp <<= 8;
                    tmp |= (uint64_t)rx.data[7-i];
                }
                if(this->ecu_ms.import_frames(tmp, rx.identifier, now)) {
                } else if (this->esp_ecu.import_frames(tmp, rx.identifier, now)) {
                } else if (this->ewm_ecu.import_frames(tmp, rx.identifier, now)) {
                } else if (this->misc_ecu.import_frames(tmp, rx.identifier, now)) {
                } else {} // TODO handle ISOTP endpoints
                vTaskDelay(1 / portTICK_PERIOD_MS); // EGS52's CAN C network is approx 250msgs/sec so we can afford to do this!
            } else {
                vTaskDelay(2 / portTICK_PERIOD_MS);
            }
        }
        if (!have_frames) {
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
}
