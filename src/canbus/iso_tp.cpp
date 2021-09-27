//
// Created by ashcon on 9/9/21.
//

#include "iso_tp.h"
#include "esp_log.h"
#include <stdlib.h>
#include <cstring>

IsoTpServer::IsoTpServer(uint16_t rx_id, uint16_t tx_id, uint8_t bs, uint8_t st_min) {
    this->bs = bs;
    this->st_min = st_min;
    this->tx_id = tx_id;
    this->rx_id = rx_id;
    this->rx_queue = xQueueCreate(10, sizeof(can_message_t));
    this->tx_queue = xQueueCreate(10, sizeof(can_message_t));
}

IsoTpServer::~IsoTpServer() {
    vTaskDelete(this->task_handler);
}

void IsoTpServer::create_server_task() {
    if (this->task_handler == nullptr) {
        xTaskCreate(__start_server_task, "ISO-TP", 32768, this, 5, this->task_handler);
    }
}

void IsoTpServer::__start_server_task(void *_this) {
    static_cast<IsoTpServer*>(_this)->server_loop();
}

void IsoTpServer::server_loop() {
    ESP_LOGD(LOG_TAG, "ISO-TP server started. Listening to CAN ID 0x%04X, sending on CAN ID 0x%04X", this->rx_id, this->tx_id);
    can_message_t tmp;
    while(true) {
        if (xQueueReceive(this->rx_queue, &tmp, 1)) {
            ESP_LOGD(LOG_TAG, "Incoming data: 0x%04X, [%02X %02X %02X %02X %02X %02X %02X %02X]", 
                tmp.identifier, 
                tmp.data[0],   
                tmp.data[1],
                tmp.data[2],   
                tmp.data[3],
                tmp.data[4],   
                tmp.data[5],
                tmp.data[6],   
                tmp.data[7]
            );
        }
        vTaskDelay(10);
    }
    vTaskDelete(this->task_handler);
}

void IsoTpServer::push_frame(can_message_t *msg) {
    can_message_t clone;
    std::memcpy(&clone, msg, sizeof(can_message_t));
    xQueueSend(this->rx_queue, (void*)&clone, 2);
}

bool IsoTpServer::pop_frame(can_message_t *dest) {
    if (uxQueueMessagesWaiting(this->tx_queue) > 0) {
        if (xQueueReceive(this->tx_queue, dest, 2) == ESP_OK) {
            return true;
        }
    }
    return false;
}
