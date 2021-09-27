//
// Created by ashcon on 9/9/21.
//
#ifndef ULTIMATE_NAG52_FW_ISO_TP_H
#define ULTIMATE_NAG52_FW_ISO_TP_H

#include <cstdint>
#include "driver/can.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define LOG_TAG "ISO_TP"

class IsoTpServer {
public:
    IsoTpServer(uint16_t rx_id, uint16_t tx_id, uint8_t bs, uint8_t st_min);
    ~IsoTpServer();
    void push_frame(can_message_t *msg);
    bool pop_frame(can_message_t *dest);
    void create_server_task();
private:
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    uint16_t rx_id;
    static void __start_server_task(void *_this);
    TaskHandle_t* task_handler = nullptr;
    void server_loop();
    uint8_t mailbox_id;
    uint8_t bs;
    uint8_t st_min;
    uint16_t tx_id;
};


#endif //ULTIMATE_NAG52_FW_ISO_TP_H
