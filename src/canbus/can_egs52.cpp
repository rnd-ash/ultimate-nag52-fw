#include "can_egs52.h"
#include "driver/can.h"
#include "pins.h"

Egs52Can::Egs52Can(const char* name, uint8_t tx_time_ms)
    : AbstractCan(name, tx_time_ms)
{
    // Firstly try to init CAN
    ESP_LOGI("EGS52_CAN", "CAN constructor called");
    can_general_config_t gen_config = CAN_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, CAN_MODE_NORMAL);
    gen_config.rx_queue_len = 10;
    gen_config.tx_queue_len = 6;
    can_timing_config_t timing_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t res;
    res = can_driver_install(&gen_config, &timing_config, &filter_config);
    if (res != ESP_OK) {
        ESP_LOGE("EGS52_CAN", "CAN_DRIVER_INSTALL FAILED!: %s", esp_err_to_name(res));
    }
    res = can_start();
    if (res != ESP_OK) {
        ESP_LOGE("EGS52_CAN", "CAN_START FAILED!: %s", esp_err_to_name(res));
    }
    // CAN is OK!

    // Set default values
    this->gs218.set_GZC(GS_218h_GZC::G_SNV);
    this->gs218.set_GIC(GS_218h_GIC::G_SNV);

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
        can_stop();
        can_driver_uninstall();
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

void Egs52Can::get_shifter_position_ewm() {
    
}

EngineType Egs52Can::get_engine_type() {
    MS_608 ms608;
    // This signal can be valid for up to 1000ms
    if (this->ecu_ms.get_MS_608(esp_timer_get_time(), 1000*1000, &ms608)) {
        ESP_LOGI("GET_ENGINE_TYPE", "%d %llX", (uint8_t)ms608.get_FCOD_KAR(), ms608.raw);
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
    
}

void Egs52Can::set_is_all_wheel_drive(bool is_4wd) {
    
}

void Egs52Can::set_wheel_torque(uint16_t t) {
    
}

void Egs52Can::set_shifter_position(ShifterPosition pos) {
    
}

void Egs52Can::set_gearbox_ok(bool is_ok) {
    
}

void Egs52Can::set_torque_request(TorqueRequest request) {
    
}

void Egs52Can::set_requested_torque(uint16_t torque_nm) {
    
}

void Egs52Can::set_error_check_status(SystemStatusCheck ssc) {
    
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
    can_message_t tx;
    tx.data_length_code = 8; // Always
    while(true) {
        /**
         * TX order of EGS52:
         * GS_338
         * GS_218
         * GS_418
         */
        start_time = esp_timer_get_time() / 1000;
        tx.identifier = GS_338_CAN_ID;
        to_bytes(gs338.raw, tx.data);
        can_transmit(&tx, 5);
        tx.identifier = GS_218_CAN_ID;
        to_bytes(gs218.raw, tx.data);
        can_transmit(&tx, 5);
        tx.identifier = GS_418_CAN_ID;
        to_bytes(gs418.raw, tx.data);
        can_transmit(&tx, 5);
        // Todo handle additional ISOTP communication
        taken = (esp_timer_get_time() / 1000) - start_time;
        if (taken < this->tx_time_ms) {
            vTaskDelay(this->tx_time_ms-taken / portTICK_PERIOD_MS);
        }
    }
}

[[noreturn]]
void Egs52Can::rx_task_loop() {
    can_message_t rx;
    can_status_info_t can_status;
    uint64_t now;
    uint64_t tmp;
    uint8_t i;
    while(true) {
        can_get_status_info(&can_status);
        bool have_frames  = false;
        while(can_status.msgs_to_rx > 0) {
            have_frames = true;
            if (can_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK && rx.data_length_code != 0 && rx.flags == 0) {
                now = esp_timer_get_time();
                tmp = 0;
                
                for(i = 0; i < 8; i++) {
                    tmp <<= 8;
                    tmp |= (uint64_t)rx.data[7-i];
                }
                bool located = false;
                if(this->ecu_ms.import_frames(tmp, rx.identifier, now)) {
                    located = true;
                } else if (this->esp_ecu.import_frames(tmp, rx.identifier, now)) {
                    located = true;
                } else if (this->ewm_ecu.import_frames(tmp, rx.identifier, now)) {
                    located = true;
                } else if (this->misc_ecu.import_frames(tmp, rx.identifier, now)) {
                    located = true;
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