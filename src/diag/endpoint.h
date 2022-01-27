#ifndef _DIAG_ENDPOINT_H__
#define _DIAG_ENDPOINT_H__

#include <stdint.h>
#include <driver/uart.h>
#include "esp_log.h"
#include "string.h"

#define DIAG_MSG_SIZE 512+4
#define DIAG_MSG_TX_SIZE (512*2)+6

typedef struct {
    uint16_t id;
    uint16_t data_size;
    uint8_t data[512]; // 512B messages max (EGS52/3 is always max 256 bytes)
} DiagMessage;

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

            e = uart_driver_install(0, DIAG_MSG_SIZE, DIAG_MSG_SIZE, 0, nullptr, 0);
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
        char read_buffer[DIAG_MSG_SIZE];
        uint16_t read_pos;
        QueueHandle_t uart_queue;
};

/**
 * @brief Endpoint for ISO-TP communication with OBD readers
 * 
 */
class CanEndpoint: public AbstractEndpoint {
    
};


#endif