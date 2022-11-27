#include "can_egs_basic.h"
#include "driver/twai.h"
#include "pins.h"

EgsBasicCan::EgsBasicCan(const char* name, uint8_t tx_time_ms)
    : AbstractCan(name, tx_time_ms)
{
    // Firstly try to init CAN
    ESP_LOG_LEVEL(ESP_LOG_INFO, "EGS_BASIC_CAN", "CAN constructor called");
    twai_general_config_t gen_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    gen_config.intr_flags = ESP_INTR_FLAG_IRAM; // Set TWAI interrupt to IRAM (Enabled in menuconfig)!
    gen_config.rx_queue_len = 32;
    gen_config.tx_queue_len = 32;
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t res;
    res = twai_driver_install(&gen_config, &timing_config, &filter_config);
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS_BASIC_CAN", "TWAI_DRIVER_INSTALL FAILED!: %s", esp_err_to_name(res));
    }
    res = twai_start();
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS_BASIC_CAN", "TWAI_START FAILED!: %s", esp_err_to_name(res));
    }
    this->can_init_ok = true;
}

bool EgsBasicCan::begin_tasks() {
    if (!this->can_init_ok) { // Cannot init tasks if CAN is dead!
        return false;
    }
    // Prevent starting again
    if (this->rx_task == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "EGS_BASIC_CAN", "Starting CAN Rx task");
        if (xTaskCreate(this->start_rx_task_loop, "EGS_BASIC_CAN_RX", 8192, this, 5, this->rx_task) != pdPASS) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS_BASIC_CAN_TX", "CAN Rx task creation failed!");
            return false;
        }
    }
    if (this->tx_task == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "EGS_BASIC_CAN", "Starting CAN Tx task");
        if (xTaskCreate(this->start_tx_task_loop, "EGS_BASIC_CAN_TX", 8192, this, 5, this->tx_task) != pdPASS) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS_BASIC_CAN_TX", "CAN Tx task creation failed!");
            return false;
        }
    }
    return true; // Ready!
}

EgsBasicCan::~EgsBasicCan()
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

WheelData EgsBasicCan::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData EgsBasicCan::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData EgsBasicCan::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData EgsBasicCan::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

ShifterPosition EgsBasicCan::get_shifter_position_ewm(uint64_t now, uint64_t expire_time_ms) {
    return ShifterPosition::SignalNotAvaliable;
}

EngineType EgsBasicCan::get_engine_type(uint64_t now, uint64_t expire_time_ms) {
    return EngineType::Unknown;
}

bool EgsBasicCan::get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool EgsBasicCan::get_kickdown(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

uint8_t EgsBasicCan::get_pedal_value(uint64_t now, uint64_t expire_time_ms) {
    return 0;
}

int EgsBasicCan::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    return 0;
}

int EgsBasicCan::get_driver_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    return 0;
}

int EgsBasicCan::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    return 0;
}

int EgsBasicCan::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    return 0;
}

PaddlePosition EgsBasicCan::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    return PaddlePosition::None;
}

int16_t EgsBasicCan::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    return INT16_MAX; // UNDEFINED
}

int16_t EgsBasicCan::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) { // TODO
    return INT16_MAX; // UNDEFINED
}

uint16_t EgsBasicCan::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    return UINT16_MAX; // UNDEFINED
}

bool EgsBasicCan::get_is_starting(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool EgsBasicCan::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

bool EgsBasicCan::get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

void EgsBasicCan::set_clutch_status(ClutchStatus status) {
}

void EgsBasicCan::set_actual_gear(GearboxGear actual) {
}

void EgsBasicCan::set_target_gear(GearboxGear target) {
}

void EgsBasicCan::set_safe_start(bool can_start) {
}

void EgsBasicCan::set_gearbox_temperature(uint16_t temp) {
}

void EgsBasicCan::set_input_shaft_speed(uint16_t rpm) {
}

void EgsBasicCan::set_is_all_wheel_drive(bool is_4wd) {
}

void EgsBasicCan::set_wheel_torque(uint16_t t) {
}

void EgsBasicCan::set_shifter_position(ShifterPosition pos) {
}

void EgsBasicCan::set_gearbox_ok(bool is_ok) {
}

void EgsBasicCan::set_torque_request(TorqueRequest request) {
}

void EgsBasicCan::set_requested_torque(uint16_t torque_nm) {
}

void EgsBasicCan::set_error_check_status(SystemStatusCheck ssc) {
}


void EgsBasicCan::set_turbine_torque_loss(uint16_t loss_nm) {
}

void EgsBasicCan::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
}

void EgsBasicCan::set_drive_profile(GearboxProfile p) {
}

void EgsBasicCan::set_race_start(bool race_start) {
}

void EgsBasicCan::set_solenoid_pwm(uint16_t duty, SolenoidName s) {
}

void EgsBasicCan::set_display_msg(GearboxMessage msg) {  
}

void EgsBasicCan::set_wheel_torque_multi_factor(float ratio) {
}

[[noreturn]]
void EgsBasicCan::tx_task_loop() {
    while(true) {
        vTaskDelay(this->tx_time_ms / portTICK_PERIOD_MS);
    }
}

[[noreturn]]
void EgsBasicCan::rx_task_loop() {
    twai_message_t rx;
    twai_status_info_t can_status;
    while(true) {
        twai_get_status_info(&can_status);
        uint8_t f_count  = can_status.msgs_to_rx;
        if (f_count == 0) {
            vTaskDelay(4 / portTICK_PERIOD_MS); // Wait for buffer to have at least 1 frame
        } else { // We have frames, read them
            for(uint8_t x = 0; x < f_count; x++) { // Read all frames
                if (twai_receive(&rx, pdMS_TO_TICKS(0)) == ESP_OK && rx.data_length_code != 0 && rx.flags == 0) {
                    if (this->diag_rx_id != 0 && rx.identifier == this->diag_rx_id) {
                        // ISO-TP Diag endpoint
                        if (this->diag_rx_queue != nullptr && rx.data_length_code == 8) {
                            // Send the frame
                            if (xQueueSend(*this->diag_rx_queue, rx.data, 0) != pdTRUE) {
                                ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS_BASIC_CAN","Discarded ISO-TP endpoint frame. Queue send failed");
                            }
                        }
                    } 
                }
            }
            vTaskDelay(2 / portTICK_PERIOD_MS); // Reset watchdog here
        }
    }
}