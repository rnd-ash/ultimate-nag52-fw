#ifndef DIAG_ENDPOINT_H
#define DIAG_ENDPOINT_H

#include <stdint.h>
#include <driver/uart.h>
#include "esp_log.h"
#include "string.h"
#include "canbus/can_hal.h"
#include "esp_core_dump.h"
#include "tcu_maths.h"
#include "driver/twai.h"
#include "esp_err.h"

// #define DIAG_CAN_MAX_SIZE 4095 // ISO-TP Maximum
static const uint16_t DIAG_CAN_MAX_SIZE = 4095u; // ISO-TP Maximum for EGS

struct DiagMessage
{
    uint16_t id;
    uint16_t data_size;
    uint8_t data[DIAG_CAN_MAX_SIZE]; // 512B messages max (EGS52/3 is always max 256 bytes)
};

struct CanEndpointMsg
{
    uint8_t data[DIAG_CAN_MAX_SIZE];
    uint16_t curr_pos;
    uint16_t max_pos;
};

/**
 * @brief Abstract endpoint
 *
 */
class AbstractEndpoint
{
public:
    AbstractEndpoint(void){};
    virtual void send_data(DiagMessage *msg); // Blocking operation
    virtual bool read_data(DiagMessage *dest);
    virtual esp_err_t init_state();
};

/**
 * @brief Endpoint for USB communication with ultimate-nag52 custom USB util
 *
 */
class UsbEndpoint : public AbstractEndpoint
{
public:
    explicit UsbEndpoint();
    void send_data(DiagMessage *msg) override;
    bool read_data(DiagMessage *dest) override;
    esp_err_t init_state() override;

private:
    // NOTE TO SELF
    // Every USB MSG:
    // {ID: 0x07E0, Data: [0x00, 0x11, 0x22, 0x33]} = '#07E100112233\n'
    // Read msg size: 6 bytes: USB message size: 14 = (Read size *2) + 2
    char *read_buffer;
    char *write_buffer;
    uint16_t read_pos;
    bool clear_to_send = false;
    int data_size;
    int line_idx;
    int max_bytes_left;
    int to_read;
    size_t length;
    esp_err_t status;
};

/**
 * @brief Endpoint for ISO-TP communication with OBD readers
 *
 */
class CanEndpoint : public AbstractEndpoint
{
public:
    explicit CanEndpoint(EgsBaseCan *can_layer);
    void send_data(DiagMessage *msg) override;
    bool read_data(DiagMessage *dest) override;
    esp_err_t init_state() override;
    static void start_iso_tp(void *_this)
    {
        static_cast<CanEndpoint *>(_this)->iso_tp_server_loop();
    }

private:
    void process_single_frame(DiagCanMessage msg);
    void process_start_frame(DiagCanMessage msg);
    void process_multi_frame(DiagCanMessage msg);
    void process_flow_control(DiagCanMessage msg);

    [[noreturn]] void iso_tp_server_loop();
    EgsBaseCan *can;
    QueueHandle_t rx_queue;
    // QueueHandle_t tx_queue;
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
    esp_err_t status;

    bool send_to_twai(DiagCanMessage msg);
};

#endif