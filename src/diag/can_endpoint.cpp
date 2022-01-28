#include "endpoint.h"
#include "kwp2000_defines.h"

CanEndpoint::CanEndpoint(AbstractCan* can_layer) {
    this->can = can_layer;
    this->tx_queue = xQueueCreate(KWP_CAN_BS, sizeof(DiagCanMessage));
    this->rx_queue = xQueueCreate(KWP_CAN_BS, sizeof(DiagCanMessage));
    can_layer->register_diag_queue(&this->rx_queue, KWP_ECU_RX_ID, &this->tx_queue, KWP_ECU_TX_ID);
}

void CanEndpoint::send_data(DiagMessage* msg) {
    if (msg->data_size < 7) {
        DiagCanMessage tx;
        tx[0] = msg->data_size;
        memcpy(&tx[1], msg->data, msg->data_size);
        if (xQueueSend(this->tx_queue, tx, 0) != pdTRUE) {
            ESP_LOGE("CAN_ENDPOINT", "Queue send failed");
        }
    } else {
        ESP_LOGE("CAN_ENDPOINT", "Sending > 7 byte ISO-TP messages TODO");
    }
}

bool CanEndpoint::read_data(DiagMessage* dest) {
    DiagCanMessage msg;
    if (xQueueReceive(this->rx_queue, msg, 0) == pdTRUE) {
        if ((msg[0] & 0xF0) == 0x00) {
            // Easy!
            dest->id = KWP_ECU_RX_ID;
            dest->data_size = msg[0];
            memcpy(dest->data, &msg[1], msg[0]);
            return true;
        } else {
            ESP_LOGE("CAN_ENDPOINT", "TODO ISO-TP PCI %02X", msg[0]);
        }
    }
    return false;
}