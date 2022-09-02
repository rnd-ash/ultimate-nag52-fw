#ifndef _DIAG_ENDPOINT_H__
#define _DIAG_ENDPOINT_H__

#include <stdint.h>
#include <driver/uart.h>
#include "esp_log.h"
#include "string.h"
#include "canbus/can_hal.h"
#include "esp_core_dump.h"
#include "tcm_maths.h"
#include "driver/twai.h"

#define DIAG_CAN_MAX_SIZE 4095 // ISO-TP Maximum

const char HEX_DEF[17] = "0123456789ABCDEF";

static const uint_fast8_t LOOKUP[256] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f 
};

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

inline uint8_t hexToByte(char* x) {
    return LOOKUP[(uint8_t)x[0]] << 4 | LOOKUP[(uint8_t)x[1]];
}

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

const static size_t UART_MSG_SIZE = 6+(2*DIAG_CAN_MAX_SIZE);

class UsbEndpoint: public AbstractEndpoint {
    public:
        UsbEndpoint(bool can_use_spiram) : AbstractEndpoint() {
            esp_err_t e;
            this->allocation_psram = can_use_spiram;
            e = uart_driver_install(0, UART_MSG_SIZE/2, UART_MSG_SIZE/2, 0, nullptr, 0);
            if (e != ESP_OK) {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint","Error installing UART driver: %s", esp_err_to_name(e));
                return;
            }
            if (this->allocation_psram) {
                this->read_buffer = (char*)heap_caps_malloc(UART_MSG_SIZE, MALLOC_CAP_SPIRAM);
                this->write_buffer = (char*)heap_caps_malloc(UART_MSG_SIZE, MALLOC_CAP_SPIRAM);
            } else {
                this->read_buffer = (char*)malloc(UART_MSG_SIZE);
                this->write_buffer = (char*)malloc(UART_MSG_SIZE);
            }
            if (this->read_buffer == nullptr) {
                return;
            }
            uart_flush(0);
            this->read_pos = 0;
        }

        void send_data(DiagMessage* msg) override {
            //esp_log_level_set("*", ESP_LOG_NONE);
            this->write_buffer[0] = '#';
            this->write_buffer[1] = HEX_DEF[(msg->id >> 12) & 0x0F];
            this->write_buffer[2] = HEX_DEF[(msg->id >> 8) & 0x0F];
            this->write_buffer[3] = HEX_DEF[(msg->id >> 4) & 0x0F];
            this->write_buffer[4] = HEX_DEF[msg->id & 0x0F];
            sprintf(&this->write_buffer[0], "#%04X", msg->id);
            for (uint16_t i = 0; i < msg->data_size; i++) {
                this->write_buffer[5+(i*2)] = HEX_DEF[(msg->data[i] >> 4) & 0x0F];
                this->write_buffer[6+(i*2)] = HEX_DEF[msg->data[i] & 0x0F];
            }
            this->write_buffer[(msg->data_size*2)+5] = '\n';
            uart_write_bytes(0, &this->write_buffer[0], (msg->data_size*2)+6);
            //esp_log_level_set("*", ESP_LOG_INFO);
        }

        int find_char(char targ, size_t from, size_t to) {
            for(size_t i = from; i < to; i++) {
                if(this->read_buffer[i] == targ) { return i; }
            }
            return -1;
        }

        bool read_data(DiagMessage* dest) override {
            this->length = 0;
            uart_get_buffered_data_len(0, &length);
            if (length != 0) {
                max_bytes_left = UART_MSG_SIZE - this->read_pos;
                to_read = MIN(length, max_bytes_left);
                uart_read_bytes(0, &this->read_buffer[this->read_pos], to_read, 0);
                if (this->read_buffer[0] != '#') {
                    // Discard
                    ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Corrupt incoming msg. Ignoring, rb[0] was '%02X', peak is '%02X'", this->read_buffer[0], this->read_buffer[1]);
                    this->read_pos = 0;
                    return false;
                }
                line_idx = find_char('\n', this->read_pos, this->read_pos+to_read);
                if (line_idx == -1) {
                    // No new line, full msg has not come in yet
                    this->read_pos += to_read;
                    return false;
                }
                if(line_idx % 2 == 0) {
                    ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Corrupt incoming msg line IDX was %d. Ignoring", line_idx);
                    this->read_pos = 0;
                    return false;
                }
                this->read_pos += to_read;
                this->data_size = MIN(DIAG_CAN_MAX_SIZE, (line_idx-5) / 2);
                // INDEX DATA
                // 0 - #
                // 1-5 - ID
                // 6..n - CAN DATA
                // n - \n
                // Copy ID
                dest->id = (hexToByte(&this->read_buffer[1]) & 0xFF << 8) | hexToByte(&this->read_buffer[3]);
                // Now copy data
                for (uint16_t i = 0; i < data_size*2; i+=2) {
                    dest->data[i/2] = hexToByte(&this->read_buffer[5+i]);
                }
                dest->data_size = data_size;
                // Circular move buffer (memove is faster!)
                memmove(&this->read_buffer[0], &this->read_buffer[line_idx], this->read_pos+to_read-line_idx);
                this->read_pos -= (line_idx+1);
                return true;
            }
            return false;
        }
    private:
        // NOTE TO SELF
        // Every USB MSG:
        // {ID: 0x07E0, Data: [0x00, 0x11, 0x22, 0x33]} = '#07E100112233\n'
        // Read msg size: 6 bytes: USB message size: 14 = (Read size *2) + 2
        char* read_buffer;
        char* write_buffer;
        uint16_t read_pos;
        bool clear_to_send = false;
        bool allocation_psram = false;
        int data_size;
        int line_idx;
        int max_bytes_left;
        int to_read;
        size_t length;
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
    //QueueHandle_t tx_queue;
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
    uint8_t tx_stmin = 20;
    uint8_t frames_received = 0;
    twai_message_t tx_can;
    uint8_t tx_count = 0;

    bool send_to_twai(DiagCanMessage msg);
};


#endif