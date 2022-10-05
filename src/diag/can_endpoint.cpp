#include "endpoint.h"
#include "kwp2000_defines.h"
#include "driver/twai.h"

const DiagCanMessage FLOW_CONTROL = {0x30, KWP_CAN_BS, KWP_CAN_ST_MIN, 0, 0, 0, 0, 0};
const DiagCanMessage FLOW_CONTROL_BUSY = {0x31, 0, 0, 0, 0, 0, 0, 0};
const DiagCanMessage FLOW_CONTROL_OVERFLOW = {0x32, 0, 0, 0, 0, 0, 0, 0};

#define ISO_TP_TIMEOUT 2000

CanEndpoint::CanEndpoint(AbstractCan* can_layer) {
    this->can = can_layer;
    memset(&tx_can, 0x00, sizeof(twai_message_t));
    this->tx_can.data_length_code = 8;
    this->tx_can.identifier = KWP_ECU_TX_ID;
    this->rx_queue = xQueueCreate(20, sizeof(DiagCanMessage)); // Queue for receiving CAN frames

    this->send_msg_queue = xQueueCreate(2, sizeof(CanEndpointMsg)); // Queue for sent messages out of KWP server
    this->read_msg_queue = xQueueCreate(2, sizeof(CanEndpointMsg)); // Queue for read messages to be pulled by KWP server
    this->is_sending = false;
    this->clear_to_send = false;
    this->is_receiving = false;
    this->last_rx_time = 0;
    this->last_tx_time = 0;
    can_layer->register_diag_queue(&this->rx_queue, KWP_ECU_RX_ID);
}

bool CanEndpoint::send_to_twai(DiagCanMessage msg) {
    memcpy(tx_can.data, msg, 8);
    return twai_transmit(&this->tx_can, 5) == ESP_OK;
}

void CanEndpoint::send_data(DiagMessage* msg) {
    this->tmp.curr_pos = 0;
    this->tmp.max_pos = msg->data_size;
    memcpy(this->tmp.data, msg->data, msg->data_size);
    if (xQueueSend(this->send_msg_queue, &this->tmp, 0) != pdTRUE) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "CanEndpoint", "Tx queue is full!?");
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
    DiagCanMessage rx;
    esp_err_t res;
    uint64_t now = esp_timer_get_time() / 1000;
    while(1) {
        now = esp_timer_get_time() / 1000;
        while (xQueueReceive(this->rx_queue, rx, 0) == pdTRUE) {
            this->last_rx_time = now;
            switch (rx[0] & 0xF0) { // Check incoming frame PCI
                case 0x00: // Single frame
                    this->process_single_frame(rx);
                    break;
                case 0x10: // Start multi-frame
                    this->process_start_frame(rx);
                    break;
                case 0x20: // Continued multi-frame
                    this->process_multi_frame(rx);
                    break;
                case 0x30: // Flow control
                    this->process_flow_control(rx);
                    break;
                default:
                    ESP_LOG_LEVEL(ESP_LOG_ERROR, "CAN_ENDPOINT", "Invalid ISO-TP PCI %02X", rx[0]);
                    break;
            }
            if (esp_timer_get_time()/1000 - now > 1) {
                break;
            }
        }

        // Check if we have stuff to send
        if (!this->is_sending) {
            if (xQueueReceive(this->send_msg_queue, &this->tx_msg, 0) == pdTRUE) {
                if (tx_msg.max_pos <= 7) { // 1 frame send
                    memset(tx_can.data, 0xCC, 8);
                    tx_can.data[0] = tx_msg.max_pos;
                    memcpy(&tx_can.data[1], tx_msg.data, tx_msg.max_pos);
                    res = twai_transmit(&tx_can, 5);
                    if (res != ESP_OK) {
                        ESP_LOG_LEVEL(ESP_LOG_ERROR, "CAN_ENDPOINT", "Could not send CAN frame %s", esp_err_to_name(res));
                    }
                } else { // Multi frame send, send the first frame and await flow control
                    this->is_sending = true; // We are sending now
                    this->clear_to_send = false; // And not clear to send
                    this->tx_pci = 0x21;
                    tx_can.data[0] = 0x10 | ((tx_msg.max_pos >> 8) & 0x0F);
                    tx_can.data[1] = tx_msg.max_pos & 0xFF; // Only one byte for len
                    memcpy(&tx_can.data[2], &tx_msg.data, 6); // Copy first 6 bytes
                    tx_msg.curr_pos = 6;
                    res = twai_transmit(&tx_can, 5);
                    if (res != ESP_OK) {
                        ESP_LOG_LEVEL(ESP_LOG_ERROR, "CAN_ENDPOINT", "Could not send CAN frame %s", esp_err_to_name(res));
                        this->is_sending = false; // Abort send
                    } else {
                        this->last_tx_time = now;
                    }
                }
            }
        }
    
        if (is_sending && clear_to_send && (now-this->last_tx_time >= KWP_CAN_ST_MIN)) {
            uint8_t max_cpy = tx_msg.max_pos-tx_msg.curr_pos;
            if (max_cpy > 7) {
                max_cpy = 7;
            }
            if (max_cpy < 7) {
                memset(tx_can.data, 0xCC, 8); // So we pad the frame with zeros
            }
            tx_can.data[0] = this->tx_pci;
            memcpy(&tx_can.data[1], &this->tx_msg.data[tx_msg.curr_pos], max_cpy);
            res = twai_transmit(&tx_can, 5);
            if (res != ESP_OK) {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "CAN_ENDPOINT", "Could not send CAN frame %s", esp_err_to_name(res));
                this->is_sending = false; // Abort send
                continue;
            } else {
                this->tx_count++;
                this->last_tx_time = now;
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
                continue;
            }
            if (this->tx_bs != 0 && this->tx_count >= this->tx_bs) {
                this->clear_to_send = false; // Await new block
            }
        }

        if (is_receiving && (now - this->last_rx_time) > ISO_TP_TIMEOUT) {
            // Timeout, mark as not receiving anymore
            ESP_LOG_LEVEL(ESP_LOG_WARN, "CAN_ENDPOINT", "Timeout waiting for rest of ISO-TP message");
            this->is_receiving = false;
        }

        if (is_sending && (now - this->last_tx_time) > ISO_TP_TIMEOUT) {
            ESP_LOG_LEVEL(ESP_LOG_WARN, "CAN_ENDPOINT", "Timeout sending rest of ISO-TP message");
            this->is_sending = false;
        }
        if (is_receiving || is_sending) { // Rx
            vTaskDelay(2);
        } else { // Idle
             vTaskDelay(25);
        }
    }
}

void CanEndpoint::process_single_frame(DiagCanMessage msg) {
    CanEndpointMsg m;
    m.max_pos = msg[0];
    memcpy(m.data, &msg[1], msg[0]);
    if (xQueueSend(this->read_msg_queue, &m, 0) != pdTRUE) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "CanEndpoint_psf", "Tx queue is full!?");
    }
}

void CanEndpoint::process_start_frame(DiagCanMessage msg) {
    if (this->is_receiving) {
        send_to_twai((uint8_t*)FLOW_CONTROL_BUSY);
        return;
    }
    uint16_t size = (msg[0] & 0x0F) << 8 | msg[1];
    if (size > DIAG_CAN_MAX_SIZE) {
        send_to_twai((uint8_t*)FLOW_CONTROL_OVERFLOW);
        return;
    }
    // Not busy receiving and message size fits
    this->is_receiving = true;
    this->rx_msg.curr_pos = 6;
    this->rx_msg.max_pos = size;
    this->frames_received = 0;
    memcpy(rx_msg.data, &msg[2], 6);
    if (!this->send_to_twai((uint8_t*)FLOW_CONTROL)) {
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
        this->frames_received++;
        if (rx_msg.curr_pos >= rx_msg.max_pos) {
            // Done!
            this->is_receiving = false;
            if (xQueueSend(this->read_msg_queue, &this->rx_msg, 0) != pdTRUE) {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "CanEndpoint_psf", "Tx queue is full!?");
            }
        } else if ((this->frames_received >= KWP_CAN_BS) && KWP_CAN_BS != 0) { // Send flow control when we overflow
            if (!this->send_to_twai((uint8_t*)FLOW_CONTROL)) {
                this->is_receiving = false; // Set if could not send Tx Frame
            }
            this->frames_received = 0;
        }

    }
}

void CanEndpoint::process_flow_control(DiagCanMessage msg) {
    ESP_LOGI("CAN_ENDPOINT", "FC Received! BS %d STMIN %d", msg[1], msg[2]);
    if (msg[0] == 0x30 && this->is_sending) {
        this->clear_to_send = true;
        this->tx_bs = msg[1];
        this->tx_stmin = msg[2];
        if (this->tx_stmin > 127) {
            this->tx_stmin = 0; // Microseconds so < 1ms
        }
        this->tx_count = 0;
        this->last_tx_time = esp_timer_get_time()/1000; // To avoid timeouts
    }
}