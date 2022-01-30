#ifndef _DIAG_ENDPOINT_H__
#define _DIAG_ENDPOINT_H__

#include <stdint.h>
#include <driver/uart.h>
#include "esp_log.h"
#include "string.h"
#include "canbus/can_hal.h"

#define DIAG_CAN_MAX_SIZE 256

typedef struct {
    uint16_t id;
    uint8_t data_size;
    uint8_t data[DIAG_CAN_MAX_SIZE]; // 512B messages max (EGS52/3 is always max 256 bytes)
} DiagMessage;

typedef struct {
    uint8_t data[DIAG_CAN_MAX_SIZE];
    uint8_t curr_pos;
    uint8_t max_pos;
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
        UsbEndpoint() : AbstractEndpoint() {
            esp_err_t e;

            e = uart_driver_install(0, 515, 515, 0, nullptr, 0);
            if (e != ESP_OK) {
                ESP_LOGE("USBEndpoint","Error installing UART driver: %s", esp_err_to_name(e));
                return;
            }
            uart_flush(0);
            this->read_pos = 0;
        }

        void send_data(DiagMessage* msg) override {
            // 2 chars per byte, 4 bytes for (CID), 2 bytes for '#' and '\n'
            int size = (msg->data_size*2)+6;
            char* buffer = (char*)malloc(size);
            if (buffer == nullptr) {
                ESP_LOGE("USBEndpoint", "Failed to allocate buffer for Tx DiagMessage!");
                return;
            }
            int len = 0;
            len = sprintf(len+buffer, "#%04X", msg->id);
            for (uint16_t i = 0; i < msg->data_size; i++) {
                len += sprintf(buffer+len, "%02X", msg->data[i]);
            }
            buffer[len] = '\n';
            uart_write_bytes(0, buffer, size);
            free(buffer);
        }

        bool read_data(DiagMessage* dest) override {
            size_t length = 0;
            uart_get_buffered_data_len(0, &length);
            if (length != 0) {
                while (uart_read_bytes(0, &this->read_buffer[this->read_pos], 1, 2) == 1) {
                    if (this->read_buffer[this->read_pos] == '\n') {
                        if (this->read_buffer[0] == '#' && this->read_pos % 2 != 0) { // Check if odd (Even would be length, -1 as we haven't yet called read_pos++)
                            uint16_t size = (this->read_pos-1) / 2;
                            uint8_t* buffer = (uint8_t*)malloc(size); // How many bytes we actually have
                            char tmp[3] = {0,0,0};
                            for (uint16_t i = 0; i < size*2; i+=2) {
                                strncpy(tmp, &this->read_buffer[1+i], 2);
                                buffer[i/2] = strtol(tmp, nullptr, 16);
                            }
                            dest->data_size = size-2;
                            dest->id = buffer[0] << 8 | buffer[1];
                            memcpy(dest->data, &buffer[2], size-2);
                            free(buffer); // Don't forget to free memory!
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
        char read_buffer[DIAG_CAN_MAX_SIZE+6];
        uint16_t read_pos;
        QueueHandle_t uart_queue;
        bool clear_to_send = false;
        bool is_sending = false;
        uint8_t pci = 0x20;
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