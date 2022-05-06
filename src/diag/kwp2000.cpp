#include "kwp2000.h"
#include <esp_ota_ops.h>
#include <string>
#include <time.h>
#include "diag_data.h"
#include "egs_emulation.h"
#include "kwp_utils.h"

typedef struct {
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t week;
} ECU_Date;

uint8_t bcd_to_hex(char c) {
    switch (c) {
        case '0':
            return 0x0;
        case '1':
            return 0x1;
        case '2':
            return 0x2;
        case '3':
            return 0x3;
        case '4':
            return 0x4;
        case '5':
            return 0x5;
        case '6':
            return 0x6;
        case '7':
            return 0x7;
        case '8':
            return 0x8;
        case '9':
            return 0x9;
        case 'A':
            return 0xA;
        case 'B':
            return 0xB;
        case 'C':
            return 0xC;
        case 'D':
            return 0xD;
        case 'E':
            return 0xE;
        default:
            return 0xF;
    }
}

ECU_Date fw_date_to_bcd(char* date) {
    uint8_t month = 0x01;
    uint8_t month_base_10 = 0;
    if (strncmp("Jan", date, 3) == 0) {
        month = 0x01;
        month_base_10 = 1;
    } else if (strncmp("Feb", date, 3) == 0) {
        month = 0x02;
        month_base_10 = 2;
    } else if (strncmp("Mar", date, 3) == 0) {
        month = 0x03;
        month_base_10 = 3;
    } else if (strncmp("Apr", date, 3) == 0) {
        month = 0x04;
        month_base_10 = 4;
    } else if (strncmp("May", date, 3) == 0) {
        month = 0x05;
        month_base_10 = 5;
    } else if (strncmp("Jun", date, 3) == 0) {
        month = 0x06;
        month_base_10 = 6;
    } else if (strncmp("Jul", date, 3) == 0) {
        month = 0x07;
        month_base_10 = 7;
    } else if (strncmp("Aug", date, 3) == 0) {
        month = 0x08;
        month_base_10 = 8;
    } else if (strncmp("Sep", date, 3) == 0) {
        month = 0x09;
        month_base_10 = 9;
    } else if (strncmp("Oct", date, 3) == 0) {
        month = 0x10;
        month_base_10 = 10;
    } else if (strncmp("Nov", date, 3) == 0) {
        month = 0x11;
        month_base_10 = 11;
    } else if (strncmp("Dec", date, 3) == 0) {
        month = 0x12;
        month_base_10 = 12;
    } else {
        month_base_10 = 99;
        month = 0x99; //??
    }

    uint8_t day_base_10 =((date[4] - '0') * 10) + (date[5]-'0'); // Hacky way
    uint8_t day = ((bcd_to_hex(date[4]) & 0x0f) << 4) | bcd_to_hex(date[5]);
    uint8_t year = ((bcd_to_hex(date[9]) & 0x0f) << 4) | bcd_to_hex(date[10]);
    uint8_t year_base_10 =((date[9] - '0') * 10) + (date[10]-'0'); // Hacky way
    
    struct tm time;
    memset(&time, 0, sizeof(time));
    char timebuf[4];
    time.tm_mday = day_base_10;
    time.tm_year = 100 + year_base_10;
    time.tm_mon = month_base_10-1;
    mktime(&time);
    strftime(timebuf, 4, "%W", &time);
    uint8_t week = ((bcd_to_hex(timebuf[0]) & 0x0f) << 4) | bcd_to_hex(timebuf[1]);

    return ECU_Date {
        .day = day,
        .month = month,
        .year = year,
        .week = week
    };
}

Kwp2000_server::Kwp2000_server(AbstractCan* can_layer, Gearbox* gearbox) {
    // Init SPIRAM (We will need this!)
    this->next_tp_time = 0;
    this->session_mode = SESSION_DEFAULT;
    this->usb_diag_endpoint = new UsbEndpoint(true);
    this->reboot_pending = false;
    this->can_layer = can_layer;
    this->gearbox_ptr = gearbox;
    this->can_endpoint = new CanEndpoint(can_layer);
    // Start ISO-TP endpoint
    xTaskCreatePinnedToCore(can_endpoint->start_iso_tp, "ISO_TP_DIAG", 8192, this->can_endpoint, 5, nullptr, 0);
    init_perfmon();
}

void Kwp2000_server::make_diag_neg_msg(uint8_t sid, uint8_t nrc) {
    /*
    this->tx_msg.id = KWP_ECU_TX_ID;
    this->tx_msg.data_size = 3;
    this->tx_msg.data[0] = 0x7F;
    this->tx_msg.data[1] = sid;
    this->tx_msg.data[2] = nrc;
    */
    global_make_diag_neg_msg(&this->tx_msg, sid, nrc);
    this->send_resp = true;
}

void Kwp2000_server::make_diag_pos_msg(uint8_t sid, const uint8_t* resp, uint16_t len) {
    /*
    if (len + 2 > DIAG_CAN_MAX_SIZE) {
        make_diag_neg_msg(sid, NRC_GENERAL_REJECT);
        return;
    }
    this->tx_msg.id = KWP_ECU_TX_ID;
    this->tx_msg.data_size = len+1;
    this->tx_msg.data[0] = sid+0x40;
    memcpy(&this->tx_msg.data[1], resp, len);
    */
    global_make_diag_pos_msg(&this->tx_msg, sid, resp, len);
    this->send_resp = true;
}

void Kwp2000_server::make_diag_pos_msg(uint8_t sid, uint8_t pid, const uint8_t* resp, uint16_t len) {
    /*
    if (len + 3 > DIAG_CAN_MAX_SIZE) {
        make_diag_neg_msg(sid, NRC_GENERAL_REJECT);
        return;
    }
    this->tx_msg.id = KWP_ECU_TX_ID;
    this->tx_msg.data_size = len+2;
    this->tx_msg.data[0] = sid+0x40;
    this->tx_msg.data[1] = pid;
    memcpy(&this->tx_msg.data[2], resp, len);
    */
    global_make_diag_pos_msg(&this->tx_msg, sid, pid, resp, len);
    this->send_resp = true;
}

int Kwp2000_server::allocate_routine_args(uint8_t* src, uint8_t arg_len) {
    free(this->running_routine_args);
    return 0;
}

void Kwp2000_server::server_loop() {
    this->send_resp = false;
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
            //ESP_LOG_BUFFER_HEX_LEVEL("KWP_READ_MSG", this->rx_msg.data, this->rx_msg.data_size, esp_log_level_t::ESP_LOG_INFO);
            if (this->rx_msg.data_size == 0) {
                continue; // Huh?
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
                case SID_READ_DATA_LOCAL_IDENT:
                    this->process_read_data_local_ident(args_ptr, args_size);
                    break;
                case SID_WRITE_DATA_BY_LOCAL_IDENT:
                    this->process_write_data_by_local_ident(args_ptr, args_size);
                    break;
                case SID_READ_MEM_BY_ADDRESS:
                    this->process_read_mem_address(args_ptr, args_size);
                    break;
                case SID_READ_ECU_IDENT:
                    this->process_read_ecu_ident(args_ptr, args_size);
                    break;
                case SID_TESTER_PRESENT:
                    this->process_tester_present(args_ptr, args_size);
                    break;
                case SID_START_ROUTINE_BY_LOCAL_IDENT:
                    this->process_start_routine_by_local_ident(args_ptr, args_size);
                    break;
                case SID_REQUEST_ROUTINE_RESULTS_BY_LOCAL_IDENT:
                    this->process_request_routine_resutls_by_local_ident(args_ptr, args_size);
                    break;
                case SID_REQ_UPLOAD:
                    this->process_request_upload(args_ptr, args_size);
                    break;
                case SID_REQ_DOWNLOAD:
                    this->process_request_download(args_ptr, args_size);
                    break;
                case SID_TRANSFER_DATA:
                    this->process_transfer_data(args_ptr, args_size);
                    break;
                case SID_TRANSFER_EXIT:
                    this->process_transfer_exit(args_ptr, args_size);
                    break;

                default:
                    ESP_LOGW("KWP_HANDLE_REQ", "Requested SID %02X is not supported", rx_msg.data[0]);
                    make_diag_neg_msg(rx_msg.data[0], NRC_SERVICE_NOT_SUPPORTED);
                    break;
            }

        }
        if (this->send_resp) {
            //ESP_LOGI("KWP", "Sending msg of %d size", tx_msg.data_size);
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
            vTaskDelay(50); // Wait for message to send (Specifically on CAN)
            esp_restart();
        }
        if (this->session_mode == SESSION_DEFAULT && this->flash_handler != nullptr) {
            delete this->flash_handler; // Remove flash handler
            this->flash_handler = nullptr;
        }
        this->cpu_usage = get_cpu_usage();
        if ((
            this->session_mode == SESSION_EXTENDED ||
            this->session_mode == SESSION_REPROGRAMMING ||
            this->session_mode == SESSION_CUSTOM_UN52)
        ) {
            vTaskDelay(5);
        } else {
            vTaskDelay(50);
        }
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
                if (!is_shifter_passive(this->can_layer)) {
                    // P or R, we CANNOT reset the ECU!
                    make_diag_neg_msg(SID_ECU_RESET, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
                    return;
                }
                this->reboot_pending = true;
                make_diag_pos_msg(SID_ECU_RESET, nullptr, 0);
            } else {
                make_diag_neg_msg(SID_ECU_RESET, NRC_REQUEST_OUT_OF_RANGE);
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
    // Any diagnostic session
    if (arg_len != 1) {
        make_diag_neg_msg(SID_READ_ECU_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_info;
    esp_ota_get_partition_description(running, &running_info);
    if (args[0] == 0x86) {
        ECU_Date date = fw_date_to_bcd(running_info.date);
        uint8_t daimler_ident_data[16];
        memset(daimler_ident_data, 0x00, 16);
        // Part number
        daimler_ident_data[0] = 0x01;
        daimler_ident_data[1] = 0x23;
        daimler_ident_data[2] = 0x45;
        daimler_ident_data[3] = 0x67;
        daimler_ident_data[4] = 0x89;
        // ECU Hardware date
        daimler_ident_data[5] = date.week;
        daimler_ident_data[6] = date.year;
        // ECU Software date
        daimler_ident_data[7] = date.week;
        daimler_ident_data[8] = date.year;
        daimler_ident_data[9] = SUPPLIER_ID;
        daimler_ident_data[10] = DIAG_VARIANT_CODE >> 8;
        daimler_ident_data[11] = DIAG_VARIANT_CODE & 0xFF;
        daimler_ident_data[13] = date.year;
        daimler_ident_data[14] = date.month;
        daimler_ident_data[15] = date.day;
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x86, daimler_ident_data, 16);
    } else if (args[0] == 0x87) { // Daimler and Mitsubishi compatible identification
        ECU_Date date = fw_date_to_bcd(running_info.date);
        uint8_t ident_data[19];
        memset(ident_data, 0x00, 19);
        ident_data[0] = 0x00; // TODO ECU origin
        ident_data[1] = SUPPLIER_ID;
        ident_data[2] = DIAG_VARIANT_CODE >> 8;
        ident_data[3] = DIAG_VARIANT_CODE & 0xFF;
        ident_data[5] = 0x00;// HW version
        ident_data[6] = 0x00;// HW version
        ident_data[7] = date.day;// SW version
        ident_data[8] = date.month;// SW version
        ident_data[9] = date.year;// SW version
        ident_data[10] = '0'; // Part number to end
        ident_data[11] = '1'; // Part number to end
        ident_data[12] = '2'; // Part number to end
        ident_data[13] = '3'; // Part number to end
        ident_data[14] = '4'; // Part number to end
        ident_data[15] = '5'; // Part number to end
        ident_data[16] = '6'; // Part number to end
        ident_data[17] = '7'; // Part number to end
        ident_data[18] = '8'; // Part number to end
        //ident_data[19] = '9'; // Part number to end
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x87, ident_data, 19);
    } else if (args[0] == 0x88) { // VIN original
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x88, (const uint8_t*)"ULTIMATENAG52ESP0", 17);
    } else if (args[0] == 0x89) { // Diagnostic variant code
        int d = DIAG_VARIANT_CODE;
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x89, (const uint8_t*)d, 4);
    } else if (args[0] == 0x90) { // VIN current
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x90, (const uint8_t*)"ULTIMATENAG52ESP0", 17);
    } else {
        make_diag_neg_msg(SID_READ_ECU_IDENT, NRC_REQUEST_OUT_OF_RANGE);
    }
}

void Kwp2000_server::process_read_data_local_ident(uint8_t* args, uint16_t arg_len) {
    if (arg_len != 1) {
        make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    if (args[0] >= 0x80 && args[0] <= 0x9F) { // ECU Ident
        this->process_read_ecu_ident(args, arg_len); // Modify the SID byte in pos/neg response to be SID_READ_DATA_LOCAL_IDENT
        if(this->tx_msg.data[0] == 0x7F) {
            this->tx_msg.data[1] = SID_READ_DATA_LOCAL_IDENT;
        } else {
            this->tx_msg.data[0] = SID_READ_DATA_LOCAL_IDENT+0x40;
        }
    } else if (args[0] == 0xE1) { // ECU Serial number
        uint8_t mac[6] = {0};
        esp_efuse_mac_get_default(mac);
        char resp[13];
        sprintf(resp, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0xE1, (const uint8_t*)resp, 12);
    } else if (args[0] == RLI_GEARBOX_SENSORS) {
        DATA_GEARBOX_SENSORS r = get_gearbox_sensors(this->gearbox_ptr);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_GEARBOX_SENSORS, (uint8_t*)&r, sizeof(DATA_GEARBOX_SENSORS));
    } else if (args[0] == RLI_SOLENOID_STATUS) {
        DATA_SOLENOIDS r = get_solenoid_data();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_SOLENOID_STATUS, (uint8_t*)&r, sizeof(DATA_SOLENOIDS));
    } else if (args[0] == RLI_CAN_DATA_DUMP) {
        DATA_CANBUS_RX r = get_rx_can_data(egs_can_hal);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_CAN_DATA_DUMP, (uint8_t*)&r, sizeof(DATA_CANBUS_RX));
    } else if (args[0] == RLI_SYS_USAGE) {
        DATA_SYS_USAGE r = get_sys_usage();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_SYS_USAGE, (uint8_t*)&r, sizeof(DATA_SYS_USAGE));
    } else if (args[0] == RLI_COREDUMP_SIZE) {
        COREDUMP_INFO r = get_coredump_info();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_COREDUMP_SIZE, (uint8_t*)&r, sizeof(COREDUMP_INFO));
    } else if (args[0] == RLI_TCM_CONFIG) {
        TCM_CORE_CONFIG r = get_tcm_config();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_TCM_CONFIG, (uint8_t*)&r, sizeof(TCM_CORE_CONFIG));
    }
    else {
        // EGS52 emulation
#ifdef EGS52_MODE
        if (args[0] == RLI_31) {
            RLI_31_DATA r = get_rli_31(egs_can_hal);
            return make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_31, (uint8_t*)&r, sizeof(RLI_31_DATA));
        }
#endif
        make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_REQUEST_OUT_OF_RANGE);
    }
    
}
void Kwp2000_server::process_read_data_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_read_mem_address(uint8_t* args, uint16_t arg_len) {
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (arg_len != 4 && arg_len != 5) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint8_t* address;
    if (arg_len == 4) {
       address = (uint8_t*)(0x40070000+((args[2] << 16) | (args[1] << 8) | args[0])); // Raw address to read from
    } else {
        address = (uint8_t*)(0x40070000+((args[3] << 24) | (args[2] << 16) | (args[1] << 8) | args[0])); // Raw address to read from 4 byte
    }
    if (address + args[arg_len-1] >= (uint8_t*)0x400BFFFF) { // Address too big (Not in SRAM 0 or SRAM1)!
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    ESP_LOGI("RMA","Pointer: %p", address);
    make_diag_pos_msg(SID_READ_MEM_BY_ADDRESS, address, args[arg_len-1]); // Copy args[3] len bytes from address into positive message
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
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52 && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (this->routine_running) {
        // Already running!
        make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
        return;
    }
    if (arg_len == 1) {
        if (args[0] == ROUTINE_SOLENOID_TEST) {
            if (gearbox_ptr->sensor_data.engine_rpm == 0 && gearbox_ptr->sensor_data.input_rpm == 0) {
                this->routine_running = true;
                this->routine_id = ROUTINE_SOLENOID_TEST;
                xTaskCreate(Kwp2000_server::launch_solenoid_test, "RT_SOL_TEST", 2048, this, 5, &this->routine_task);
                uint8_t resp[1] = {ROUTINE_SOLENOID_TEST};
                make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, resp, 1);
            } else {
                make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
            }
        } else if (args[0] == ROUTINE_FLASH_CHECK) {
            if (this->flash_handler != nullptr) {
                this->tx_msg = this->flash_handler->on_request_verification(args, arg_len);
                this->send_resp = true;
                return;
            } else {
                make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
            }
        } else {
            make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        }
    } else if (arg_len == 3) {
        if (args[0] == ROUTINE_SOLENOID_TEST) {
            // Args[1] -> Freq/10
            // Args[2] -> Time/10
        } else {
            make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        }
    } else {
        make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
}
void Kwp2000_server::process_stop_routine_by_local_ident(uint8_t* args, uint16_t arg_len) {
    
}
void Kwp2000_server::process_request_routine_resutls_by_local_ident(uint8_t* args, uint16_t arg_len) {
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52  && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_REQUEST_ROUTINE_RESULTS_BY_LOCAL_IDENT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (this->routine_running) {
        // Already running!
        make_diag_neg_msg(SID_REQUEST_ROUTINE_RESULTS_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
        return;
    }
    if (arg_len != 1) {
        make_diag_neg_msg(SID_REQUEST_ROUTINE_RESULTS_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    if (args[0] != this->routine_id) {
        make_diag_neg_msg(SID_REQUEST_ROUTINE_RESULTS_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
        return;
    }
    make_diag_pos_msg(SID_REQUEST_ROUTINE_RESULTS_BY_LOCAL_IDENT, this->routine_result, this->routine_results_len);
}


void Kwp2000_server::process_request_download(uint8_t* args, uint16_t arg_len) {
    // Valid session only
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (this->flash_handler != nullptr) {
        delete this->flash_handler;
    }
    // Make a new flash handler!
    this->flash_handler = new Flasher(this->can_layer, this->gearbox_ptr);
    this->tx_msg = this->flash_handler->on_request_download(args, arg_len);
    this->send_resp = true;
}

void Kwp2000_server::process_request_upload(uint8_t* args, uint16_t arg_len) {
    // Valid session only
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (this->flash_handler != nullptr) {
        delete this->flash_handler;
    }
    this->flash_handler = new Flasher(this->can_layer, this->gearbox_ptr);
    this->tx_msg = this->flash_handler->on_request_upload(args, arg_len);
    this->send_resp = true;
}

void Kwp2000_server::process_transfer_data(uint8_t* args, uint16_t arg_len) {
    // Valid session only
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_TRANSFER_DATA, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (this->flash_handler == nullptr) {
        make_diag_neg_msg(SID_TRANSFER_DATA, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
        return;
    } else {
        // Flasher will do the rest for us
        this->tx_msg = this->flash_handler->on_transfer_data(args, arg_len);
        this->send_resp = true;
    }
}

void Kwp2000_server::process_transfer_exit(uint8_t* args, uint16_t arg_len) {
    // Valid session only
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_TRANSFER_EXIT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (this->flash_handler == nullptr) {
        make_diag_neg_msg(SID_TRANSFER_EXIT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
        return;
    }  else {
        // Flasher will do the rest for us
        this->tx_msg = this->flash_handler->on_transfer_exit(args, arg_len);
        this->send_resp = true;
    }
}

void Kwp2000_server::process_write_data_by_local_ident(uint8_t* args, uint16_t arg_len) {
    if (
        this->session_mode == SESSION_EXTENDED ||
        this->session_mode == SESSION_REPROGRAMMING ||
        this->session_mode == SESSION_STANDBY ||
        this->session_mode == SESSION_CUSTOM_UN52
    ) {
        if (args[0] == RLI_TCM_CONFIG) {
            if (arg_len-1 != sizeof(TCM_CORE_CONFIG)) {
                make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            } else {
                // TCM Core config size ok
                TCM_CORE_CONFIG cfg;
                memcpy(&cfg, &args[1], sizeof(TCM_CORE_CONFIG));
                uint8_t res = set_tcm_config(cfg);
                if (res == 0x00) {
                    make_diag_pos_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, RLI_TCM_CONFIG, nullptr, 0);
                } else {
                    make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, res);
                }
            }
        } else {
            make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_REQUEST_OUT_OF_RANGE);
        }
    } else  {
        make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
    }
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


void Kwp2000_server::run_solenoid_test() {
    this->gearbox_ptr->diag_inhibit_control();
    ESP_LOGI("RT_SOL_TEST", "Starting solenoid test");
    this->routine_results_len = 7;
    // Routine results format
    // 6 bytes (1 per solenoid)
    // 0x00 - OK
    // 0x0F - Overcurrent when 0 (Short detected)
    // 0xF0 - Undercurrent when open (Possible sensor fault)
    memset(this->routine_result, 0, 7);
    this->routine_result[0] = this->routine_id;
    sol_mpc->write_pwm_12_bit(0);
    sol_spc->write_pwm_12_bit(0);
    sol_tcc->write_pwm_12_bit(0);
    sol_y3->write_pwm_12_bit(0);
    sol_y4->write_pwm_12_bit(0);
    sol_y5->write_pwm_12_bit(0);
    vTaskDelay(1000);
    if (sol_mpc->get_current_estimate() > 100) {
        ESP_LOGE("RT_SOL_TEST", "MPC overcurrent");
        this->routine_result[1] = 0x0F;
    }
    if (sol_spc->get_current_estimate() > 100) {
        ESP_LOGE("RT_SOL_TEST", "SPC overcurrent");
        this->routine_result[2] = 0x0F;
    }
    if (sol_tcc->get_current_estimate() > 100) {
        ESP_LOGE("RT_SOL_TEST", "TCC overcurrent");
        this->routine_result[3] = 0x0F;
    }
    if (sol_y3->get_current_estimate() > 100) {
        ESP_LOGE("RT_SOL_TEST", "Y3 overcurrent");
        this->routine_result[4] = 0x0F;
    }
    if (sol_y4->get_current_estimate() > 100) {
        ESP_LOGE("RT_SOL_TEST", "Y4 overcurrent");
        this->routine_result[5] = 0x0F;
    }
    if (sol_y5->get_current_estimate() > 100) {
        ESP_LOGE("RT_SOL_TEST", "Y5 overcurrent");
        this->routine_result[6] = 0x0F;
    }
    // Now test each solenoid 1x1
    sol_mpc->write_pwm_12_bit(2048);
    vTaskDelay(600);
    if (sol_mpc->get_current_estimate() < 500) {
        ESP_LOGE("RT_SOL_TEST", "MPC undercurrent %d", sol_mpc->get_current_estimate());
        this->routine_result[1] |= 0xF0;
        //goto cleanup;
    }
    sol_mpc->write_pwm_12_bit(0);
    sol_spc->write_pwm_12_bit(2048);
    vTaskDelay(600);
    if (sol_spc->get_current_estimate() < 500) {
        ESP_LOGE("RT_SOL_TEST", "SPC undercurrent %d", sol_spc->get_current_estimate());
        this->routine_result[2] |= 0xF0;
        //goto cleanup;
    }
    sol_spc->write_pwm_12_bit(0);
    sol_tcc->write_pwm_12_bit(2048);
    vTaskDelay(600);
    if (sol_tcc->get_current_estimate() < 500) {
        ESP_LOGE("RT_SOL_TEST", "TCC undercurrent %d", sol_tcc->get_current_estimate());
        this->routine_result[3] |= 0xF0;
        //goto cleanup;
    }
    sol_tcc->write_pwm_12_bit(0);
    sol_y3->write_pwm_12_bit(2048);
    vTaskDelay(600);
    if (sol_y3->get_current_estimate() < 500) {
        ESP_LOGE("RT_SOL_TEST", "Y3 undercurrent %d", sol_y3->get_current_estimate());
        this->routine_result[4] |= 0xF0;
        //goto cleanup;
    }
    sol_y3->write_pwm_12_bit(0);
    sol_y4->write_pwm_12_bit(2048);
    vTaskDelay(600);
    if (sol_y4->get_current_estimate() < 500) {
        ESP_LOGE("RT_SOL_TEST", "Y4 undercurrent %d", sol_y4->get_current_estimate());
        this->routine_result[5] |= 0xF0;
        //goto cleanup;
    }
    sol_y4->write_pwm_12_bit(0);
    sol_y5->write_pwm_12_bit(2048);
    vTaskDelay(600);
    if (sol_y5->get_current_estimate() < 500) {
        ESP_LOGE("RT_SOL_TEST", "Y5 undercurrent %d", sol_y5->get_current_estimate());
        this->routine_result[6] |= 0xF0;
        //goto cleanup;
    }
    sol_y5->write_pwm_12_bit(0);
    ESP_LOGI("RT_SOL_TEST", "Cleaning up");
    this->routine_running = false;
    this->gearbox_ptr->diag_regain_control();
    vTaskDelete(nullptr);
}