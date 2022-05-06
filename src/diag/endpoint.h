#ifndef _DIAG_ENDPOINT_H__
#define _DIAG_ENDPOINT_H__

#include <stdint.h>
#include <driver/uart.h>
#include "esp_log.h"
#include "string.h"
#include "canbus/can_hal.h"
#include "esp_core_dump.h"
#include "tcm_maths.h"

#define DIAG_CAN_MAX_SIZE 2048

typedef struct {
    uint16_t id;
    uint16_t data_size;
    uint8_t data[DIAG_CAN_MAX_SIZE]; // 512B messages max (EGS52/3 is always max 256 bytes)
} DiagMessage;

typedef struct {
    uint8_t data[DIAG_CAN_MAX_SIZE];
    uint16_t curr_pos;
    uint16_t max_pos;
} CanEndpointMsg;

/**
 * @brief Abstract endpoint
 * 
 */
class AbstractEndpoint {
public:
    AbstractEndpoint(){};
    virtual void send_data(DiagMessage* msg); // Blocking operation
    virtual bool read_data(DiagMessage* dest);
};

/**
 * @brief Endpoint for USB communication with ultimate-nag52 custom USB util
 * 
 */

#define UART_NUM 
class UsbEndpoint: public AbstractEndpoint {
    public:
        UsbEndpoint(bool can_use_spiram) : AbstractEndpoint() {
            const size_t read_buffer_size = 5+(2*DIAG_CAN_MAX_SIZE);
            esp_err_t e;
            this->allocation_psram = can_use_spiram;
            e = uart_driver_install(0, 3+(2*DIAG_CAN_MAX_SIZE), 3+(2*DIAG_CAN_MAX_SIZE), 0, nullptr, 0);
            if (e != ESP_OK) {
                ESP_LOGE("USBEndpoint","Error installing UART driver: %s", esp_err_to_name(e));
                return;
            }
            if (this->allocation_psram) {
                this->read_buffer = (char*)heap_caps_malloc(read_buffer_size, MALLOC_CAP_SPIRAM);
            } else {
                this->read_buffer = (char*)malloc(read_buffer_size);
            }
            if (this->read_buffer == nullptr) {
                return;
            }
            uart_flush(0);
            this->read_pos = 0;
        }

        void send_data(DiagMessage* msg) override {
            char buf[6];
            sprintf(buf, "#%04X", msg->id);
            uart_write_bytes(0, buf, 5);
            for (uint16_t i = 0; i < msg->data_size; i++) {
                sprintf(buf, "%02X", msg->data[i]);
                uart_write_bytes(0, buf, 2);
            }
            uart_write_bytes(0, "\n", 1);
        }

        bool read_data(DiagMessage* dest) override {
            size_t length = 0;
            uart_get_buffered_data_len(0, &length);
            if (length != 0) {
                while (uart_read_bytes(0, &this->read_buffer[this->read_pos], 1, 2) == 1) {
                    if (this->read_buffer[this->read_pos] == '\n') {
                        if (this->read_buffer[0] == '#' && this->read_pos % 2 != 0) { // Check if odd (Even would be length, -1 as we haven't yet called read_pos++)
                            uint16_t data_size = MIN(DIAG_CAN_MAX_SIZE, (this->read_pos-5) / 2);
                            char tmp[3] = {0,0,0};
                            // INDEX DATA
                            // 0 - #
                            // 1-5 - ID
                            // 6..n - CAN DATA
                            // n - \n
                            // Copy ID
                            strncpy(tmp, &this->read_buffer[1], 2);
                            dest->id = strtol(tmp, nullptr, 16) & 0xFF << 8;
                            strncpy(tmp, &this->read_buffer[3], 2);
                            dest->id |= strtol(tmp, nullptr, 16) & 0xFF;
                            // Now copy data
                            for (uint16_t i = 0; i < data_size*2; i+=2) {
                                strncpy(tmp, &this->read_buffer[5+i], 2);
                                dest->data[i/2] = strtol(tmp, nullptr, 16);
                            }
                            dest->data_size = data_size;
                            this->read_pos = 0;
                            return true;
                        } else {
                            ESP_LOGE("USBEndpoint", "Corrupt incoming msg. Ignoring");
                            this->read_pos = 0;
                            return false;
                        }
                    }
                    this->read_pos++;
                }
            }
            return false;
        }
    private:
        // NOTE TO SELF
        // Every USB MSG:
        // {ID: 0x07E0, Data: [0x00, 0x11, 0x22, 0x33]} = '#07E100112233\n'
        // Read msg size: 6 bytes: USB message size: 14 = (Read size *2) + 2
        char* read_buffer;
        uint16_t read_pos;
        QueueHandle_t uart_queue;
        bool clear_to_send = false;
        bool is_sending = false;
        uint8_t pci = 0x20;
        bool allocation_psram = false;
};

/**
 * @brief Endpoint for ISO-TP communication with OBD readers
 * 
 */
class CanEndpoint: public AbstractEndpoint {
public:
    CanEndpoint(AbstractCan* can_layer);
    void send_data(DiagMessage* msg) override;
    bool read_data(DiagMessage* dest) override;
    static void start_iso_tp(void *_this) {
        static_cast<CanEndpoint*>(_this)->iso_tp_server_loop();
    }
private:

    void process_single_frame(DiagCanMessage msg);
    void process_start_frame(DiagCanMessage msg);
    void process_multi_frame(DiagCanMessage msg);
    void process_flow_control(DiagCanMessage msg);

    [[noreturn]]
    void iso_tp_server_loop();
    AbstractCan* can;
    QueueHandle_t rx_queue;
    QueueHandle_t tx_queue;
    CanEndpointMsg tx_msg;
    CanEndpointMsg rx_msg;
    QueueHandle_t read_msg_queue;
    QueueHandle_t send_msg_queue;
    CanEndpointMsg tmp;
    bool is_sending;
    bool clear_to_send;
    bool is_receiving;
    uint8_t rx_bs;
    uint8_t tx_pci = 0x20;
    uint64_t last_rx_time;
    uint64_t last_tx_time;
    uint8_t tx_bs = 8;
};


#endif