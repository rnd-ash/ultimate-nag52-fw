#include "can_hal.h"
#include "board_config.h"
#include "esp_check.h"

EgsBaseCan* egs_can_hal = nullptr;

EgsBaseCan::EgsBaseCan(const char* name, uint8_t tx_time_ms, uint32_t baud) {
    this->name = name;
    this->diag_rx_id = 0x07E1;
    this->diag_tx_id = 0x07E9;
    this->diag_rx_queue = nullptr;
    this->can_init_status = ESP_OK;
    this->tx_time_ms = tx_time_ms;

    // Firstly try to init CAN
    ESP_LOG_LEVEL(ESP_LOG_INFO, this->name, "Booting CAN Layer");
    if (nullptr == pcb_gpio_matrix) {
        ESP_LOGE(this->name, "No GPIO matrix! Cannot start CAN");
        return;
    }
    twai_general_config_t gen_config = TWAI_GENERAL_CONFIG_DEFAULT(pcb_gpio_matrix->can_tx_pin, pcb_gpio_matrix->can_rx_pin, TWAI_MODE_NORMAL);
    gen_config.intr_flags = ESP_INTR_FLAG_IRAM; // Set TWAI interrupt to IRAM (Enabled in menuconfig)!
    gen_config.rx_queue_len = 32;
    gen_config.tx_queue_len = 32;
    twai_timing_config_t timing_config{};
    switch(baud) {
        case 1000000: // 1mbps
            timing_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
        case 800000: // 800kbps
            timing_config = TWAI_TIMING_CONFIG_800KBITS();
            break;
        case 500000: // 500kbps
            timing_config = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case 250000:
            timing_config = TWAI_TIMING_CONFIG_250KBITS();
            break;
        case 125000:
            timing_config = TWAI_TIMING_CONFIG_125KBITS();
            break;
        case 100000:
            timing_config = TWAI_TIMING_CONFIG_100KBITS();
            break;
        default:
            ESP_LOGE(this->name, "Cannot set CAN baud to %d", baud);
            this->can_init_status = ESP_ERR_INVALID_ARG;
            break;
        
    }
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (this->can_init_status == ESP_OK) {
        // Init OK (Baud valid)
        this->can_init_status = twai_driver_install(&gen_config, &timing_config, &filter_config);
        if (this->can_init_status == ESP_OK) {
            this->can_init_status = twai_start();
            if (this->can_init_status == ESP_OK) {
                ESP_LOGI(this->name, "Calling setup");
            } else {
                ESP_LOGE(this->name, "Failed to start TWAI");
            } 
        } else {
            ESP_LOGE(this->name, "Failed to install TWAI driver");
        }
    }
}

EgsBaseCan::~EgsBaseCan() {
    if (this->rx_task != nullptr) {
        vTaskDelete(this->rx_task);
    }
    if (this->tx_task != nullptr) {
        vTaskDelete(this->tx_task);
    }
    // Delete CAN
    if (this->can_init_status == ESP_OK) {
        twai_stop();
        twai_driver_uninstall();
    }
}

bool EgsBaseCan::begin_tasks() {
    if (this->can_init_status != ESP_OK) {
        return false;
    }
    // Prevent starting again
    if (this->rx_task == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_INFO, this->name, "Starting CAN Rx task");
        if (xTaskCreate(this->start_rx_task_loop, "EGS_CAN_RX", 8192, this, 5, this->rx_task) != pdPASS) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, this->name, "CAN Rx task creation failed!");
            return false;
        }
    }
    if (this->tx_task == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_INFO, this->name, "Starting CAN Tx task");
        if (xTaskCreate(this->start_tx_task_loop, "EGS_CAN_TX", 4096, this, 5, this->tx_task) != pdPASS) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, this->name, "CAN Tx task creation failed!");
            return false;
        }
    }
    return true; // Ready!
}

[[noreturn]]
void EgsBaseCan::tx_task_loop() {
    while(true) {
        if (this->send_messages) {
            this->tx_frames();
        }
        vTaskDelay(this->tx_time_ms / portTICK_PERIOD_MS);
    }
}

[[noreturn]]
void EgsBaseCan::rx_task_loop() {
    twai_message_t rx;
    uint8_t i;
    uint64_t tmp;
    uint64_t now;
    while(true) {
        now = esp_timer_get_time() / 1000;
        twai_get_status_info(&this->can_status);
        uint8_t f_count  = can_status.msgs_to_rx;
        if (f_count == 0) {
            vTaskDelay(4 / portTICK_PERIOD_MS); // Wait for buffer to have at least 1 frame
        } else { // We have frames, read them
            for(uint8_t x = 0; x < f_count; x++) { // Read all frames
                if (twai_receive(&rx, pdMS_TO_TICKS(0)) == ESP_OK && rx.data_length_code != 0 && rx.flags == 0) {
                    tmp = 0;
                    for(i = 0; i < rx.data_length_code; i++) {
                        tmp |= (uint64_t)rx.data[i] << (8*(7-i));
                    }
                    this->on_rx_frame(rx.identifier, rx.data_length_code, tmp, now);
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
        this->on_rx_done(now);
    }
}