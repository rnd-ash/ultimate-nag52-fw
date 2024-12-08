#include "can_hal.h"
#include "board_config.h"
#include "esp_check.h"
#include "clock.hpp"
#include "nvs/device_mode.h"

#include "../shifter/shifter_ewm.h"
#include "../shifter/shifter_trrs.h"

EgsBaseCan *egs_can_hal = nullptr;

EgsBaseCan::EgsBaseCan(const char *name, uint8_t tx_time_ms, uint32_t baud, Shifter *shifter)
{
    this->name = name;
    this->diag_rx_id = 0x07E1;
    this->diag_tx_id = 0x07E9;
    this->diag_rx_queue = nullptr;
    this->can_init_status = ESP_OK;
    this->tx_time_ms = tx_time_ms;
    this->shifter = shifter;

    // Firstly try to init CAN
    ESP_LOG_LEVEL(ESP_LOG_INFO, this->name, "Booting CAN Layer");
    if (nullptr == pcb_gpio_matrix)
    {
        this->can_init_status = ESP_ERR_INVALID_STATE;
        ESP_LOGE(this->name, "No GPIO matrix! Cannot start CAN");
        return;
    }
    twai_general_config_t gen_config = TWAI_GENERAL_CONFIG_DEFAULT(pcb_gpio_matrix->can_tx_pin, pcb_gpio_matrix->can_rx_pin, TWAI_MODE_NORMAL);
    gen_config.intr_flags = ESP_INTR_FLAG_IRAM;
    gen_config.rx_queue_len = 32;
    gen_config.tx_queue_len = 32;
    twai_timing_config_t timing_config{};
    switch (baud)
    {
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
        ESP_LOGE(this->name, "Cannot set CAN baud to %lu", baud);
        this->can_init_status = ESP_ERR_INVALID_ARG;
        break;
    }
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (this->can_init_status == ESP_OK)
    {
        // Init OK (Baud valid)
        this->can_init_status = twai_driver_install(&gen_config, &timing_config, &filter_config);
        if (this->can_init_status == ESP_OK)
        {
            this->can_init_status = twai_start();
            if (this->can_init_status == ESP_OK)
            {
                ESP_LOGD(this->name, "Calling setup");
            }
            else
            {
                ESP_LOGE(this->name, "Failed to start TWAI");
            }
        }
        else
        {
            ESP_LOGE(this->name, "Failed to install TWAI driver");
        }
    }
    // Now set the constants for the Tx message.
    this->tx.extd = 0;
    this->tx.rtr = 0;
    this->tx.ss = 0; // Always single shot
    this->tx.self = 0;
    this->tx.dlc_non_comp = 0;
}

esp_err_t EgsBaseCan::init_state() const
{
    return this->can_init_status;
}

bool EgsBaseCan::bus_ok() const
{
    return this->can_status.state == twai_state_t::TWAI_STATE_RUNNING;
}

EgsBaseCan::~EgsBaseCan() {
    if (this->task != nullptr) {
        vTaskDelete(this->task);
    }
    // Delete CAN
    if (this->can_init_status == ESP_OK)
    {
        twai_stop();
        twai_driver_uninstall();
    }
}

bool EgsBaseCan::begin_task() {
    if (this->can_init_status != ESP_OK) {
        return false;
    }
    // Prevent starting again
    if (this->task == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, this->name, "Starting CAN task");
        if (xTaskCreate(this->start_task_loop, "EGS_CAN", 8192, this, 5, &this->task) != pdPASS) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, this->name, "CAN task creation failed!");
            return false;
        }
    }
    return true; // Ready!
}

[[noreturn]]
void EgsBaseCan::task_loop() {
    twai_message_t rx;
    uint8_t i;
    uint64_t tmp;
    uint32_t now;
    while(true) {
        now = GET_CLOCK_TIME();
        // Message Rx
        twai_get_status_info(&this->can_status);
        // Handle BUS OFF and BUS STOPPED
        if (this->can_status.state == twai_state_t::TWAI_STATE_BUS_OFF) {
            twai_initiate_recovery();
        } else if (this->can_status.state == twai_state_t::TWAI_STATE_STOPPED) {
            twai_start();
        } else if (likely(this->can_status.state == twai_state_t::TWAI_STATE_RUNNING)) {
            uint8_t f_count  = can_status.msgs_to_rx;
            for(uint8_t x = 0; x < f_count; x++) { // Read all frames
                if (twai_receive(&rx, pdMS_TO_TICKS(0)) == ESP_OK && rx.data_length_code != 0 && rx.flags == 0) {
                    if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_CANLOGGER)) {
                        // Logging mode
                        char buf[35];
                        int pos = 0;
                        pos += sprintf(buf + pos, "CF->0x%04X", (uint16_t)rx.identifier);
                        for (uint8_t i = 0; i < rx.data_length_code; i++) {
                            pos += sprintf(buf + pos, "%02X", rx.data[i]);
                        }
                        printf("%.*s\n", pos, buf);
                    } else {
                        if (this->diag_rx_id != 0 && rx.identifier == this->diag_rx_id) {
                            // ISO-TP Diag endpoint
                            if (this->diag_rx_queue != nullptr && rx.data_length_code == 8) {
                                // Send the frame
                                if (xQueueSend(*this->diag_rx_queue, rx.data, 0) != pdTRUE) {
                                    ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS_BASIC_CAN","Discarded ISO-TP endpoint frame. Queue send failed");
                                }
                            }
                        } else { // Normal message
                            tmp = 0;
                            for(i = 0; i < rx.data_length_code; i++) {
                                tmp |= (uint64_t)rx.data[i] << (8*(7-i));
                            }
                            if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {
                                // Slave mode handling
                                this->egs_slave_mode_tester.import_frames(tmp, rx.identifier, now);
                            } else {
                                this->on_rx_frame(rx.identifier, rx.data_length_code, tmp, now);
                            }
                        }
                    }
                }
            }
            // Message Tx
            if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {
                // Only Tx slave frames
                tx.data_length_code = 8;

                uint64_t solenoid = solenoid_slave_resp.raw;
                uint64_t sensors = sensors_slave_resp.raw;
                uint64_t un52 = un52_slave_resp.raw;

                tx.identifier = SOLENOID_REPORT_EGS_SLAVE_CAN_ID;
                to_bytes(solenoid, tx.data);
                twai_transmit(&tx, 5);

                tx.identifier = SENSOR_REPORT_EGS_SLAVE_CAN_ID;
                to_bytes(sensors, tx.data);
                twai_transmit(&tx, 5);

                tx.identifier = UN52_REPORT_EGS_SLAVE_CAN_ID;
                to_bytes(un52, tx.data);
                twai_transmit(&tx, 5);
            }
            else if (this->send_messages && !CHECK_MODE_BIT_ENABLED(DEVICE_MODE_CANLOGGER))
            {
                this->tx_frames();
            }
        }
        this->on_rx_done(now);
        int taken = GET_CLOCK_TIME() - now;
        if (taken < this->tx_time_ms) {
            vTaskDelay((this->tx_time_ms - taken) / portTICK_PERIOD_MS); // Reset watchdog here
        }
    }
}

void EgsBaseCan::on_rx_done(const uint32_t now_ts)
{
    if (nullptr != shifter) {
        switch (VEHICLE_CONFIG.shifter_style)
        {
        case (uint8_t)ShifterStyle::EWM:
        {
            ShifterEwm *shifterewm = reinterpret_cast<ShifterEwm *>(shifter);
            shifterewm->set_program_button_pressed(get_profile_btn_press(now_ts), get_profile_switch_pos(now_ts));
            break;
        }
        case (uint8_t)ShifterStyle::TRRS:
        {
            ShifterTrrs *shiftertrrs = reinterpret_cast<ShifterTrrs *>(shifter);
            shiftertrrs->set_brake_is_pressed(get_is_brake_pressed(now_ts));
            shiftertrrs->set_vehicle_speed(get_front_left_wheel(now_ts), get_front_right_wheel(now_ts));
            break;
        }
        default:
            break;
        }
    }
}
