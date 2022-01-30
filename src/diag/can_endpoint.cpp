#include "endpoint.h"
#include "kwp2000_defines.h"

const DiagCanMessage FLOW_CONTROL = {0x30, KWP_CAN_BS, KWP_CAN_ST_MIN};
const DiagCanMessage FLOW_CONTROL_BUSY = {0x31, 0, 0};
const DiagCanMessage FLOW_CONTROL_OVERFLOW = {0x32, 0, 0};

#define ISO_TP_TIMEOUT 2000

CanEndpoint::CanEndpoint(AbstractCan* can_layer) {
    this->can = can_layer;
    this->tx_queue = xQueueCreate(KWP_CAN_BS, sizeof(DiagCanMessage)); // Queue for sending CAN frames
    this->rx_queue = xQueueCreate(KWP_CAN_BS, sizeof(DiagCanMessage)); // Queue for receiving CAN frames

    this->send_msg_queue = xQueueCreate(2, sizeof(CanEndpointMsg)); // Queue for sent messages out of KWP server
    this->read_msg_queue = xQueueCreate(2, sizeof(CanEndpointMsg)); // Queue for read messages to be pulled by KWP server
    this->is_sending = false;
    this->clear_to_send = false;
    this->is_receiving = false;
    this->last_rx_time = 0;
    this->last_tx_time = 0;
    can_layer->register_diag_queue(&this->rx_queue, KWP_ECU_RX_ID, &this->tx_queue, KWP_ECU_TX_ID);
}

void CanEndpoint::send_data(DiagMessage* msg) {
    this->tmp.curr_pos = 0;
    this->tmp.max_pos = msg->data_size;
    memcpy(this->tmp.data, msg->data, msg->data_size);
    if (xQueueSend(this->send_msg_queue, &this->tmp, 0) != pdTRUE) {
        ESP_LOGE("CanEndpoint", "Tx queue is full!?");
    }
}

bool CanEndpoint::read_data(DiagMessage* dest) {
    if (xQueueReceive(this->read_msg_queue, &this->tmp, 0) == pdTRUE) {
        dest->id = KWP_ECU_RX_ID;
        dest->data_size = this->tmp.max_pos;
        memcpy(dest->data, this->tmp.data, this->tmp.max_pos);
        return true;
    }
    return false;
}

void CanEndpoint::iso_tp_server_loop() {
    // This loop deals with sending and receiving data from ISO-TP
    DiagCanMessage msg;
    while(1) {
        if (xQueueReceive(this->rx_queue, msg, 0) == pdTRUE) {
            switch (msg[0] & 0xF0) { // Check incoming frame PCI
                case 0x00: // Single frame
                    this->process_single_frame(msg);
                    break;
                case 0x10: // Start multi-frame
                    this->process_start_frame(msg);
                    break;
                case 0x20: // Continued multi-frame
                    this->process_multi_frame(msg);
                    break;
                case 0x30: // Flow control
                    this->process_flow_control(msg);
                    break;
                default:
                    ESP_LOGE("CAN_ENDPOINT", "Invalid ISO-TP PCI %02X", msg[0]);
                    break;
            } 
        }

        // Check if we have stuff to send
        if (!this->is_sending) {
            if (xQueueReceive(this->send_msg_queue, &this->tx_msg, 0) == pdTRUE) {
                if (tx_msg.max_pos <= 7) { // 1 frame send
                    msg[0] = tx_msg.max_pos;
                    memcpy(&msg[1], tx_msg.data, tx_msg.max_pos);
                    if (xQueueSend(this->tx_queue, msg, 0) != pdTRUE) {
                        ESP_LOGE("CAN_ENDPOINT", "Could not send CAN frame to CAN_HAL");
                    }
                } else { // Multi frame send, send the first frame and await flow control
                    this->is_sending = true; // We are sending now
                    this->clear_to_send = false; // And not clear to send
                    this->tx_pci = 0x21;
                    msg[0] = 0x10;
                    msg[1] = tx_msg.max_pos; // Only one byte for len
                    memcpy(&msg[2], &tx_msg.data, 6); // Copy first 6 bytes
                    tx_msg.curr_pos = 6;
                    if (xQueueSend(this->tx_queue, msg, 0) != pdTRUE) {
                        ESP_LOGE("CAN_ENDPOINT", "Could not send CAN frame to CAN_HAL");
                        this->is_sending = false; // Abort send
                    } else {
                        this->last_tx_time = esp_timer_get_time()/1000;
                    }
                }
            }
        }
    
        if (is_sending && clear_to_send) {
            // We can send our block (Always assume BS)
            for (uint8_t i = 0; i < KWP_CAN_BS; i++) {
                uint8_t max_cpy = tx_msg.max_pos-tx_msg.curr_pos;
                if (7 < max_cpy) {
                    max_cpy = 7;
                }
                msg[0] = this->tx_pci;
                memcpy(&msg[1], &this->tx_msg.data[tx_msg.curr_pos], max_cpy);
                if (xQueueSend(this->tx_queue, msg, 0) != pdTRUE) {
                    ESP_LOGE("CAN_ENDPOINT", "Could not send CAN frame to CAN_HAL");
                    this->is_sending = false; // Abort send
                    break;
                } else {
                    this->last_tx_time = esp_timer_get_time()/1000;
                }

                tx_msg.curr_pos += 7;
                tx_pci+= 1;
                if (tx_pci == 0x30) { // Roll over PCI
                    tx_pci = 0x20;
                }
                // Check if we have sent everything
                if (tx_msg.curr_pos >= tx_msg.max_pos) {
                    this->is_sending = false;
                    this->clear_to_send = false;
                    break;
                }
            }
            this->clear_to_send = false; // Await new block
        }

        if (is_receiving && ((esp_timer_get_time()/1000) - this->last_rx_time) > ISO_TP_TIMEOUT) {
            // Timeout, mark as not receiving anymore
            ESP_LOGW("CAN_ENDPOINT", "Timeout waiting for rest of ISO-TP message");
            this->is_receiving = false;
        }

        if (is_sending && ((esp_timer_get_time()/1000) - this->last_tx_time) > ISO_TP_TIMEOUT) {
            ESP_LOGW("CAN_ENDPOINT", "Timeout sending rest of ISO-TP message");
            this->is_sending = false;
        }

        vTaskDelay(KWP_CAN_ST_MIN);
    }
}

void CanEndpoint::process_single_frame(DiagCanMessage msg) {
    CanEndpointMsg m;
    m.max_pos = msg[0];
    memcpy(m.data, &msg[1], msg[0]);
    if (xQueueSend(this->read_msg_queue, &m, 0) != pdTRUE) {
        ESP_LOGE("CanEndpoint_psf", "Tx queue is full!?");
    }
}

void CanEndpoint::process_start_frame(DiagCanMessage msg) {
    if (this->is_receiving) {
        xQueueSend(this->tx_queue, FLOW_CONTROL_BUSY, 0);
        return;
    }
    uint16_t size = (msg[0] & 0x0F) << 8 | msg[1];
    if (size > DIAG_CAN_MAX_SIZE) {
        xQueueSend(this->tx_queue, FLOW_CONTROL_OVERFLOW, 0);
        return;
    }
    // Not busy receiving and message size fits
    this->is_receiving = true;
    this->rx_msg.curr_pos = 6;
    this->rx_msg.max_pos = size;
    this->last_rx_time = esp_timer_get_time()/1000;
    memcpy(rx_msg.data, &msg[2], 6);
    if (xQueueSend(this->tx_queue, FLOW_CONTROL, 0) != pdTRUE) {
        this->is_receiving = false; // Set if could not send Tx Frame
    }
}

void CanEndpoint::process_multi_frame(DiagCanMessage msg) {
    if (this->is_receiving) {
        int max_copy = this->rx_msg.max_pos - this->rx_msg.curr_pos;
        if (7 < max_copy) {
            max_copy = 7;
        }
        memcpy(&this->rx_msg.data[rx_msg.curr_pos], &msg[1], max_copy);
        rx_msg.curr_pos += max_copy;
        if (rx_msg.curr_pos >= rx_msg.max_pos) {
            // Done!
            this->is_receiving = false;
            if (xQueueSend(this->read_msg_queue, &this->rx_msg, 0) != pdTRUE) {
                ESP_LOGE("CanEndpoint_psf", "Tx queue is full!?");
            }
        }

    }
}

void CanEndpoint::process_flow_control(DiagCanMessage msg) {
    ESP_LOGI("CAN_ENDPOINT", "FC Received!");
    if (msg[0] == 0x30 && this->is_sending) {
        this->clear_to_send = true;
        this->tx_bs = msg[1];
        this->last_tx_time = esp_timer_get_time()/1000; // To avoid timeouts
    }
}