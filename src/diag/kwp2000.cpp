#include "kwp2000.h"

Kwp2000_server::Kwp2000_server() {
    this->next_tp_time = 0;
    this->session_mode = SESSION_DEFAULT;
    this->usb_diag_endpoint = new UsbEndpoint();
}

void make_diag_neg_msg(DiagMessage* dest, uint8_t sid, uint8_t nrc) {
    dest->id = KWP_ECU_TX_ID;
    dest->data_size = 3;
    dest->data[0] = 0x7F;
    dest->data[1] = sid;
    dest->data[2] = nrc;
}

void make_diag_pos_msg(DiagMessage* dest, uint8_t sid, uint8_t* resp, uint16_t len) {
    dest->id = KWP_ECU_TX_ID;
    dest->data_size = len+1;
    dest->data[0] = sid+0x40;
    memcpy(&dest[1], resp, len);
}

void Kwp2000_server::server_loop() {
    while(1) {
        if (this->usb_diag_endpoint->read_data(&this->rx_msg)) {
            // New message! process it
            switch(rx_msg.data[0]) { // SID byte
                default:
                    ESP_LOGW("KWP_HANDLE_REQ", "Requested SID %02X is not supported", rx_msg.data[0]);
                    make_diag_neg_msg(&this->tx_msg, rx_msg.data[0], NRC_SERVICE_NOT_SUPPORTED);
                    send_resp = true;
                    break;
            }

        }
        if (this->send_resp) {
            this->usb_diag_endpoint->send_data(&tx_msg);
            this->send_resp = false;
        }
        if ((
            (this->session_mode == SESSION_EXTENDED) ||
            (this->session_mode == SESSION_REPROGRAMMING) ||
            (this->session_mode == SESSION_CUSTOM_UN52))
            && esp_timer_get_time()/1000 > this->next_tp_time
        ) {
            ESP_LOGI("KWP2000", "Tester present interval has expired, returning to default mode");
            this->session_mode = SESSION_DEFAULT;
        }
        vTaskDelay(25);
    }
}