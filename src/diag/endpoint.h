#ifndef _DIAG_ENDPOINT_H__
#define _DIAG_ENDPOINT_H__

#include <stdint.h>
#include <driver/uart.h>
#include "esp_log.h"
#include "string.h"

#define DIAG_MSG_SIZE 1024+4

typedef struct DiagMessage {
    uint16_t id;
    uint16_t data_size;
    uint8_t data[1024]; // 1KB messages max (EGS52/3 is always max 256 bytes)
};

/**
 * @brief Abstract endpoint
 * 
 */
class AbstractEndpoint {
public:
    AbstractEndpoint();
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

        }

        void send_data(DiagMessage* msg) override {
            // 2 chars per byte, 2 bytes for (CID), 2 bytes for '#' and '\n'
            int size = (msg->data_size*2)+4;
            char* buffer = (char*)malloc(size);
            if (buffer == nullptr) {
                ESP_LOGE("USBEndpoint", "Failed to allocate buffer for Tx DiagMessage!");
                return;
            }
            buffer[0] = '#';
            buffer[1] = (msg->id >> 8) & 0xFF;
            buffer[2] = msg->id & 0xFF;
            int len = 3;
            for (uint16_t i = 0; i < msg->data_size; i++) {
                sprintf(buffer+len, "%02X", msg->data[i]);
            }
            buffer[size-1] = '\n';
            uart_write_bytes(0, buffer, size);
            free(buffer);
        }

        bool read_data(DiagMessage* dest) override {
            size_t length = 0;
            uart_get_buffered_data_len(0, &length);
            if (length != 0) {
                int max_read = DIAG_MSG_SIZE-this->read_pos;
                this->read_pos += uart_read_bytes(0, &this->read_buffer[this->read_pos], max_read, 2);
                if (this->read_pos >= DIAG_MSG_SIZE) {
                    dest->id = read_buffer[0] << 8 | read_buffer[1];
                    dest->data_size = read_buffer[2] << 8 | read_buffer[3];
                    memcpy(dest->data, &read_buffer[4], dest->data_size); // Only copy what we want rather than entire 1024 data
                    this->read_pos = 0; // Reset
                    return true;
                }
            }
            return false;
        }
    private:
        uint8_t read_buffer[DIAG_MSG_SIZE];
        uint16_t read_pos;
};

/**
 * @brief Endpoint for ISO-TP communication with OBD readers
 * 
 */
class CanEndpoint: public AbstractEndpoint {
    
};


#endif