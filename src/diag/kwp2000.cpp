#include "kwp2000.h"

Kwp2000_server::Kwp2000_server(AbstractCan* can_layer, Gearbox* gearbox) {
    this->next_tp_time = 0;
    this->session_mode = SESSION_DEFAULT;
    this->usb_diag_endpoint = new UsbEndpoint();
    this->reboot_pending = false;
    this->can_layer = can_layer;
    this->gearbox_ptr = gearbox;
    this->can_endpoint = new CanEndpoint(can_layer);
}

void Kwp2000_server::make_diag_neg_msg(uint8_t sid, uint8_t nrc) {
    this->tx_msg.id = KWP_ECU_TX_ID;
    this->tx_msg.data_size = 3;
    this->tx_msg.data[0] = 0x7F;
    this->tx_msg.data[1] = sid;
    this->tx_msg.data[2] = nrc;
    this->send_resp = true;
}

void Kwp2000_server::make_diag_pos_msg(uint8_t sid, uint8_t* resp, uint16_t len) {
    this->tx_msg.id = KWP_ECU_TX_ID;
    this->tx_msg.data_size = len+1;
    this->tx_msg.data[0] = sid+0x40;
    memcpy(&this->tx_msg.data[1], resp, len);
    this->send_resp = true;
}

void Kwp2000_server::server_loop() {
    while(1) {
        bool read_msg = false;
        bool endpoint_was_usb = false;
        if (this->usb_diag_endpoint->read_data(&this->rx_msg)) {
            endpoint_was_usb = true;
            read_msg = true;
        } else if (this->can_endpoint->read_data(&this->rx_msg)) {
            endpoint_was_usb = false;
            read_msg = true;
        }
        if (read_msg) {
            if (this->rx_msg.data_size == 0) {
                break; // Huh?
            }

            // New message! process it
            uint8_t* args_ptr = &rx_msg.data[1];
            uint16_t args_size = rx_msg.data_size - 1;
            switch(rx_msg.data[0]) { // SID byte
                case SID_START_DIAGNOSTIC_SESSION:
                    this->process_start_diag_session(args_ptr, args_size);
                    break;
                case SID_ECU_RESET:
                    this->process_ecu_reset(args_ptr, args_size);
                    break;
                case SID_TESTER_PRESENT:
                    this->process_tester_present(args_ptr, args_size);
                    break;
                default:
                    ESP_LOGW("KWP_HANDLE_REQ", "Requested SID %02X is not supported", rx_msg.data[0]);
                    make_diag_neg_msg(rx_msg.data[0], NRC_SERVICE_NOT_SUPPORTED);
                    break;
            }

        }
        if (this->send_resp) {
            if (endpoint_was_usb) {
                this->usb_diag_endpoint->send_data(&tx_msg);
            } else {
                this->can_endpoint->send_data(&tx_msg);
            }
            this->send_resp = false;
        }
        if ((
            this->session_mode == SESSION_EXTENDED ||
            this->session_mode == SESSION_REPROGRAMMING ||
            this->session_mode == SESSION_CUSTOM_UN52)
            && esp_timer_get_time()/1000 > this->next_tp_time
        ) {
            ESP_LOGI("KWP2000", "Tester present interval has expired, returning to default mode");
            this->session_mode = SESSION_DEFAULT;
        }
        if (this->reboot_pending) {
            esp_restart();
        }
        vTaskDelay(50);
    }
}


void Kwp2000_server::process_start_diag_session(uint8_t* args, uint16_t arg_len) {
    if (arg_len != 1) { // Must only have 1 arg
        make_diag_neg_msg(SID_START_DIAGNOSTIC_SESSION, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    bool advance_tp_interval = true;
    switch (args[0]) {
        case SESSION_DEFAULT:
        case SESSION_PASSIVE:
        case SESSION_STANDBY:
            advance_tp_interval = false;
            break;
        case SESSION_EXTENDED:
        case SESSION_REPROGRAMMING:
        case SESSION_CUSTOM_UN52:
            break;
        default:
            // Not supported session mode!
            make_diag_neg_msg(SID_START_DIAGNOSTIC_SESSION, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
    }
    if (advance_tp_interval) {
        this->next_tp_time = (esp_timer_get_time()/1000)+KWP_TP_TIMEOUT_MS;
    }
    this->session_mode = args[0];
    make_diag_pos_msg(SID_START_DIAGNOSTIC_SESSION, &args[0], 1);
}

void Kwp2000_server::process_ecu_reset(uint8_t* args, uint16_t arg_len) {
    if (
        this->session_mode == SESSION_EXTENDED || 
        this->session_mode == SESSION_STANDBY || 
        this->session_mode == SESSION_REPROGRAMMING ||
        this->session_mode == SESSION_CUSTOM_UN52
    ) {
        // Session type OK, process the request
        if (arg_len != 1) { // Must only have 1 arg
            make_diag_neg_msg(SID_ECU_RESET, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        } else {
            // 1 arg, process the reset type
            if (args[0] == 0x01 || args[1] == 0x82) {
                this->reboot_pending = true;
                make_diag_pos_msg(SID_ECU_RESET, nullptr, 0);
            } else {
                make_diag_neg_msg(SID_ECU_RESET, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            }
        }
    } else {
        // Invalid session type, cannot reset ECU
        make_diag_neg_msg(SID_ECU_RESET, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
    }
}

void Kwp2000_server::process_clear_diag_info(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_read_status_of_dtcs(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_read_ecu_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_read_data_local_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_read_data_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_read_mem_address(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_security_access(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_disable_msg_tx(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_enable_msg_tx(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_dynamically_define_local_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_write_data_by_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_ioctl_by_local_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_start_routine_by_local_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_stop_routine_by_local_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_request_routine_resutls_by_local_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_request_download(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_request_upload(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_transfer_data(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_transfer_exit(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_write_data_by_local_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_write_mem_by_address(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_tester_present(uint8_t* args, uint16_t arg_len) {
    if (arg_len != 1) { // Must only have 1 arg
        make_diag_neg_msg(SID_TESTER_PRESENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    if (args[0] == KWP_CMD_RESPONSE_REQUIRED) {
        make_diag_pos_msg(SID_TESTER_PRESENT, nullptr, 0);
        this->next_tp_time = esp_timer_get_time()/1000 + KWP_TP_TIMEOUT_MS;
    } else if (args[1] == KWP_CMD_NO_RESPONSE_REQUIRED) {
        this->next_tp_time = esp_timer_get_time()/1000 + KWP_TP_TIMEOUT_MS;
    } else {
        make_diag_neg_msg(SID_TESTER_PRESENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
}
void Kwp2000_server::process_control_dtc_settings(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_response_on_event(uint8_t* args, uint16_t arg_len) {

}