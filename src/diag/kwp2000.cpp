#include "kwp2000.h"
#include <esp_ota_ops.h>
#include <string>
#include <time.h>
#include "diag_data.h"
#include "egs_emulation.h"
#include "kwp_utils.h"
#include "map_editor.h"
#include "esp_mac.h"
#include "models/clutch_speed.hpp"
#include "tcu_alloc.h"
#include "solenoids/solenoids.h"
#include "clock.hpp"
#include "nvs/device_mode.h"
#include "esp_flash.h"
#include "egs_calibration/calibration_structs.h"
#include "tcu_io/tcu_io.hpp"

typedef struct {
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t week;
} ECU_Date;

uint8_t decToBcd(uint8_t val)
{
  return ( (val/10*16) + (val%10) );
}

uint8_t bcd_to_hex(char c) {
    if (c >= 0x48 && c<= 0x57) { // 0-9
        return c - 48;
    } else if (c >= 0x64 && c <= 0x70) { // A-F
        return c - 65 + 10;
    } else {
        return 0x0F;
    }
}

const ECU_Date pcb_ver_to_date(TCM_EFUSE_CONFIG* cfg) {
    switch (cfg->board_ver) {
        case 1:
            return ECU_Date {
                .day = 12,
                .month = 12,
                .year = 21,
                .week = 49
            };
        case 2:
            return ECU_Date {
                .day = 07,
                .month = 07,
                .year = 22,
                .week = 27
            };
        
        case 3:
            return ECU_Date {
                .day = 12,
                .month = 12,
                .year = 22,
                .week = 49
            };
        
        default:
            return ECU_Date {
                .day = 0,
                .month = 0,
                .year = 0,
                .week = 0
            };
    }
}

ECU_Date fw_date_to_bcd(char* date) {
    uint8_t month = 0x01;
    char* month_str = &date[3];
    if (strncmp("Jan", month_str, 3) == 0) {
        month = 1;
    } else if (strncmp("Feb", month_str, 3) == 0) {
        month = 2;
    } else if (strncmp("Mar", month_str, 3) == 0) {
        month = 3;
    } else if (strncmp("Apr", month_str, 3) == 0) {
        month = 4;
    } else if (strncmp("May", month_str, 3) == 0) {
        month = 5;
    } else if (strncmp("Jun", month_str, 3) == 0) {
        month = 6;
    } else if (strncmp("Jul", month_str, 3) == 0) {
        month = 7;
    } else if (strncmp("Aug", month_str, 3) == 0) {
        month = 8;
    } else if (strncmp("Sep", month_str, 3) == 0) {
        month = 9;
    } else if (strncmp("Oct", month_str, 3) == 0) {
        month = 10;
    } else if (strncmp("Nov", month_str, 3) == 0) {
        month = 11;
    } else if (strncmp("Dec", month_str, 3) == 0) {
        month = 12;
    } else {
        month = 0x00;
    }

    uint8_t day  =((date[0] - '0') * 10) + (date[1]-'0');
    uint8_t year =((date[9] - '0') * 10) + (date[10]-'0');
    struct tm time;
    memset(&time, 0, sizeof(time));
    char timebuf[4];
    time.tm_mday = day;
    time.tm_year = 100 + year;
    time.tm_mon = month-1;
    mktime(&time);
    strftime(timebuf, 4, "%02W", &time);
    uint8_t week =((timebuf[0] - '0') * 10) + (timebuf[1]-'0'); // Hacky way
    return ECU_Date {
        .day = day,
        .month = month,
        .year = year,
        .week = week
    };
}

Kwp2000_server::Kwp2000_server(EgsBaseCan* can_layer, Gearbox* gearbox) {
    // Init SPIRAM (We will need this!)
    this->next_tp_time = 0;
    this->session_mode = SESSION_DEFAULT;
    this->usb_diag_endpoint = new UsbEndpoint();
    this->reboot_pending = false;
    this->can_layer = can_layer;
    this->gearbox_ptr = gearbox;
    this->can_endpoint = new CanEndpoint(can_layer);
    if (this->can_endpoint->init_state() == ESP_OK) {
        // Start ISO-TP endpoint
        xTaskCreatePinnedToCore(can_endpoint->start_iso_tp, "ISO_TP_DIAG", 8192, this->can_endpoint, 5, nullptr, 0);
    }
    this->supplier_id = 0x08;
    if (can_layer == nullptr || gearbox == nullptr) {
        this->diag_var_code = 0x0000;
    } else {
        switch (VEHICLE_CONFIG.egs_can_type) {
            case 1:
                this->diag_var_code = 0x0251;
                break;
            case 2:
                this->diag_var_code = 0x0252;
                break;
            case 3:
                this->diag_var_code = 0x0353;
                break;
            default:
                this->diag_var_code = 0x0000;
                break;

        }
    }
    PerfMon::init_perfmon();
}

kwp_result_t Kwp2000_server::convert_err_result(kwp_result_t in) {
    kwp_result_t out = NRC_GENERAL_REJECT;
    switch(in) {
        case NRC_UN52_ENGINE_OFF:
        case NRC_UN52_ENGINE_ON:
        case NRC_UN52_SHIFTER_ACTIVE:
        case NRC_UN52_SHIFTER_PASSIVE:
            out = NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR;
            break;
        case NRC_UN52_NO_MEM:
            out = NRC_GENERAL_REJECT;
            break;
        default:
            out = in;
            break;
    }
    return out;
}

Kwp2000_server::~Kwp2000_server() {
    if (this->flash_handler != nullptr) {
        delete this->flash_handler;
    }
}

void Kwp2000_server::make_diag_neg_msg(uint8_t sid, uint8_t nrc) {
    kwp_result_t nrc_convert = nrc;
    if (this->session_mode != SESSION_CUSTOM_UN52) {
        nrc_convert = this->convert_err_result(nrc);
    }
    global_make_diag_neg_msg(&this->tx_msg, sid, nrc_convert);
    this->send_resp = true;
}

void Kwp2000_server::make_diag_pos_msg(uint8_t sid, const uint8_t* resp, uint16_t len) {
    global_make_diag_pos_msg(&this->tx_msg, sid, resp, len);
    this->send_resp = true;
}

void Kwp2000_server::make_diag_pos_msg(uint8_t sid, uint8_t pid, const uint8_t* resp, uint16_t len) {
    global_make_diag_pos_msg(&this->tx_msg, sid, pid, resp, len);
    this->send_resp = true;
}

int Kwp2000_server::allocate_routine_args(uint8_t* src, uint8_t arg_len) {
    free(this->running_routine_args);
    return 0;
}

void Kwp2000_server::start_response_timer(uint8_t sid) {
    this->response_pending_sid = sid;
    this->cmd_recv_time = GET_CLOCK_TIME();
    this->response_pending = true;
}

void Kwp2000_server::end_response_timer() {
    this->response_pending = false;
}

void Kwp2000_server::response_timer_loop() {
    uint8_t buf[3] = {0x7F, 0x00, NRC_RESPONSE_PENDING};
    while(1) {
        if (this->response_pending && (GET_CLOCK_TIME() - this->cmd_recv_time) > KWP_RESPONSEPENDING_INTERVAL) {
            buf[1] = this->response_pending_sid;
            // Send 0x78 (Response pending)
            if (this->diag_on_usb) {
                this->usb_diag_endpoint->send_data(KWP_ECU_TX_ID, buf, 3);
            } else {
                this->can_endpoint->send_data(KWP_ECU_TX_ID, buf, 3);
            }
            this->cmd_recv_time = GET_CLOCK_TIME();
        }
        vTaskDelay(250/portTICK_PERIOD_MS);
    }
}

void Kwp2000_server::server_loop() {
    this->send_resp = false;
    while(1) {
        PerfMon::update_sample();
        uint32_t timestamp = GET_CLOCK_TIME();
        bool read_msg = false;
        if (this->usb_diag_endpoint->init_state() == ESP_OK && this->usb_diag_endpoint->read_data(&this->rx_msg)) {
            this->diag_on_usb = true;
            read_msg = true;
        } else if (this->can_endpoint->init_state() == ESP_OK && this->can_endpoint->read_data(&this->rx_msg)) {
            this->diag_on_usb = false;
            read_msg = true;
        }
        if (read_msg) {
            this->next_tp_time = timestamp+KWP_TP_TIMEOUT_MS;
            if (this->rx_msg.data_size == 0) {
                continue; // Huh?
            }

            // New message! process it
            uint8_t* args_ptr = &rx_msg.data[1];
            uint16_t args_size = rx_msg.data_size - 1;
            start_response_timer(rx_msg.data[0]);
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
                case SID_WRITE_MEM_BY_ADDRESS:
                    this->process_write_mem_by_address(args_ptr, args_size);
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
                    this->process_request_routine_results_by_local_ident(args_ptr, args_size);
                    break;
                case SID_REQ_UPLOAD:
                    this->process_request_upload(args_ptr, args_size);
                    break;
                case SID_REQ_DOWNLOAD:
                    this->process_request_download(args_ptr, args_size);
                    break;
                case SID_IOCTL_BY_LOCAL_IDENT:
                    this->process_ioctl_by_local_ident(args_ptr, args_size);
                    break;
                case SID_TRANSFER_DATA:
                    this->process_transfer_data(args_ptr, args_size);
                    break;
                case SID_TRANSFER_EXIT:
                    this->process_transfer_exit(args_ptr, args_size);
                    break;
                case SID_SHIFT_MGR_OP:
                    this->process_shift_mgr_op(args_ptr, args_size);
                    break;
                case SID_ENABLE_NORMAL_MSG_TRANSMISSION:
                    this->process_enable_msg_tx(args_ptr, args_size);
                    break;
                case SID_DISABLE_NORMAL_MSG_TRANSMISSION:
                    this->process_disable_msg_tx(args_ptr, args_size);
                    break;
                default:
                    ESP_LOG_LEVEL(ESP_LOG_WARN, "KWP_HANDLE_REQ", "Requested SID %02X is not supported, full msg was:", rx_msg.data[0]);
                    ESP_LOG_BUFFER_HEX_LEVEL("KWP_HANDLE_REQ", rx_msg.data, rx_msg.data_size, ESP_LOG_WARN);
                    make_diag_neg_msg(rx_msg.data[0], NRC_SERVICE_NOT_SUPPORTED);
                    break;
            }
        }
        end_response_timer();
        if (this->send_resp) {
            if (this->diag_on_usb) {
                this->usb_diag_endpoint->send_data(tx_msg.id, tx_msg.data, tx_msg.data_size);
            } else if (this->can_endpoint != nullptr) {
                this->can_endpoint->send_data(tx_msg.id, tx_msg.data, tx_msg.data_size);
            }
            this->send_resp = false;
        }
        if ((
            this->session_mode == SESSION_EXTENDED ||
            this->session_mode == SESSION_REPROGRAMMING ||
            this->session_mode == SESSION_CUSTOM_UN52)
            && timestamp > this->next_tp_time
        ) {
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
        if ((
            this->session_mode == SESSION_EXTENDED ||
            this->session_mode == SESSION_REPROGRAMMING ||
            this->session_mode == SESSION_CUSTOM_UN52)
        ) {
            vTaskDelay(2);
        } else {
            vTaskDelay(50);
        }
    }
}


void Kwp2000_server::process_start_diag_session(const uint8_t* args, uint16_t arg_len) {
    if (arg_len != 1) { // Must only have 1 arg
        make_diag_neg_msg(SID_START_DIAGNOSTIC_SESSION, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    switch (args[0]) {
        case SESSION_DEFAULT:
        case SESSION_PASSIVE:
        case SESSION_STANDBY:
            break;
        case SESSION_EXTENDED:
        case SESSION_REPROGRAMMING:
        case SESSION_CUSTOM_UN52:
            this->next_tp_time = GET_CLOCK_TIME()+KWP_TP_TIMEOUT_MS;
            break;
        default:
            // Not supported session mode!
            make_diag_neg_msg(SID_START_DIAGNOSTIC_SESSION, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
    }
    this->session_mode = args[0];
    make_diag_pos_msg(SID_START_DIAGNOSTIC_SESSION, &args[0], 1);
}

void Kwp2000_server::process_ecu_reset(const uint8_t* args, uint16_t arg_len) {
    if (
        this->session_mode == SESSION_EXTENDED || 
        this->session_mode == SESSION_REPROGRAMMING ||
        this->session_mode == SESSION_CUSTOM_UN52
    ) {
        // Session type OK, process the request
        if (arg_len != 1) { // Must only have 1 arg
            make_diag_neg_msg(SID_ECU_RESET, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        } else {
            // 1 arg, process the reset type
            if (args[0] == 0x01 || args[1] == 0x82) {
                if (nullptr != gearbox) {
                    if (this->can_layer != nullptr && !is_shifter_passive(this->can_layer)) {
                        // P or R, we CANNOT reset the ECU!
                        make_diag_neg_msg(SID_ECU_RESET, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
                        return;
                    }
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

void Kwp2000_server::process_read_ecu_ident(const uint8_t* args, uint16_t arg_len) {
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
        ECU_Date v_date = pcb_ver_to_date(&BOARD_CONFIG);
        uint8_t daimler_ident_data[16];
        memset(daimler_ident_data, 0x00, 16);
        // Part number
        daimler_ident_data[0] = 0x12;
        daimler_ident_data[1] = 0x23;
        daimler_ident_data[2] = 0x45;
        daimler_ident_data[3] = 0x67;
        daimler_ident_data[4] = 0x89;
        // ECU Hardware date

        daimler_ident_data[5] = decToBcd(v_date.week); //date.week;
        daimler_ident_data[6] = decToBcd(v_date.year); //date.year;
        // ECU Software date
        daimler_ident_data[7] = decToBcd(date.week);
        daimler_ident_data[8] = decToBcd(date.year);
        daimler_ident_data[9] = this->supplier_id;
        daimler_ident_data[10] = this->diag_var_code >> 8;
        daimler_ident_data[11] = this->diag_var_code & 0xFF;
        daimler_ident_data[13] = decToBcd(BOARD_CONFIG.manufacture_year);
        daimler_ident_data[14] = decToBcd(BOARD_CONFIG.manufacture_month);
        daimler_ident_data[15] = decToBcd(BOARD_CONFIG.manufacture_day);
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x86, daimler_ident_data, 16);
    } else if (args[0] == 0x87) { // Daimler and Mitsubishi compatible identification
        ECU_Date date = fw_date_to_bcd(running_info.date);
        uint8_t ident_data[19];
        memset(ident_data, 0x00, 19);
        ident_data[0] = 0x00; // TODO ECU origin
        ident_data[1] = this->supplier_id;
        ident_data[2] = this->diag_var_code >> 8;
        ident_data[3] = this->diag_var_code & 0xFF;
        ident_data[5] = 0x01;// HW version
        ident_data[6] = BOARD_CONFIG.board_ver;// HW version
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
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x88, reinterpret_cast<const uint8_t*>("ULTIMATENAG52ESP0"), 17);
    } else if (args[0] == 0x89) { // Diagnostic variant code
        int d = this->diag_var_code;
        uint8_t b[4];
        memcpy(b, &d, 4);
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x89, b, 4);
    } else if (args[0] == 0x8A) {
        char x[4];
        x[0] = 'H';
        x[1] = 'E';
        x[2] = 'L';
        x[3] = 'P';
        return make_diag_pos_msg(SID_READ_ECU_IDENT, 0x8A, (uint8_t*)&x, 4);
    } else if (args[0] == 0x90) { // VIN current
        make_diag_pos_msg(SID_READ_ECU_IDENT, 0x90, reinterpret_cast<const uint8_t*>("ULTIMATENAG52ESP0"), 17);
    } else {
        make_diag_neg_msg(SID_READ_ECU_IDENT, NRC_REQUEST_OUT_OF_RANGE);
    }
}

void Kwp2000_server::process_read_data_local_ident(uint8_t* args, uint16_t arg_len) {
    if (arg_len != 1 && (args[0] != RLI_MAP_EDITOR && args[0] != RLI_SETTINGS_EDIT)) {
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
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0xE1, reinterpret_cast<const uint8_t*>(resp), 12);
    } else if (args[0] == RLI_MAP_EDITOR) {
        // 0 - RLI
        // 1 - Map ID
        // 2 - CMD
        // 3-4 - Arg len
        // 5..n - Data
        if (arg_len < 5) {
            make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
        }
        uint8_t map_id = args[1];
        uint8_t cmd = args[2];
        uint16_t map_len_bytes = args[3] << 8 | args[4];
        if (arg_len-5 != map_len_bytes) {
            make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
        }
        uint8_t ret;
        uint8_t* buffer = nullptr;
        uint16_t read_bytes_size = 0;
        if ( cmd == MAP_CMD_READ || cmd == MAP_CMD_READ_DEFAULT || cmd == MAP_CMD_READ_EEPROM) {
            uint8_t c;
            if (cmd == MAP_CMD_READ) {
                c = MAP_READ_TYPE_MEM;
            } else if (cmd == MAP_CMD_READ_DEFAULT) {
                c = MAP_READ_TYPE_PRG;
            } else { // MAP_CMD_READ_EEPROM
                c = MAP_READ_TYPE_STO;
            }
            ret = MapEditor::read_map_data(map_id, c, &read_bytes_size, &buffer);
        } else if (cmd == MAP_CMD_READ_META) { 
            ret = MapEditor::read_map_metadata(map_id, &read_bytes_size, &buffer);
        } else {
            ret = NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
        }
        if (ret == 0) { // OK
            uint8_t* buf = static_cast<uint8_t*>(TCU_HEAP_ALLOC(2+read_bytes_size));
            if (buf == nullptr) {
                TCU_FREE(buffer); // DELETE MapEditor allocation
                make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_GENERAL_REJECT);
                return;
            }
            buf[0] = read_bytes_size & 0xFF;
            buf[1] = read_bytes_size >> 8;
            memcpy(&buf[2], buffer, read_bytes_size);
            make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, buf, 2+read_bytes_size);
            delete[] buf;
            TCU_FREE(buffer); // DELETE MapEditor allocation
            return;
        } else {
            make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, ret);
            return;
        }
    } else if (args[0] == RLI_GEARBOX_SENSORS) {
        DATA_GEARBOX_SENSORS r = get_gearbox_sensors(this->gearbox_ptr);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_GEARBOX_SENSORS, (uint8_t*)&r, sizeof(DATA_GEARBOX_SENSORS));
    } else if (args[0] == RLI_SOLENOID_STATUS) {
        DATA_SOLENOIDS r = get_solenoid_data(this->gearbox_ptr);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_SOLENOID_STATUS, (uint8_t*)&r, sizeof(DATA_SOLENOIDS));
    } else if (args[0] == RLI_CAN_DATA_DUMP) {
        DATA_CANBUS_RX r = get_rx_can_data(egs_can_hal);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_CAN_DATA_DUMP, (uint8_t*)&r, sizeof(DATA_CANBUS_RX));
    } else if (args[0] == RLI_SYS_USAGE) {
        DATA_SYS_USAGE r = get_sys_usage();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_SYS_USAGE, (uint8_t*)&r, sizeof(DATA_SYS_USAGE));
    } else if (args[0] == RLI_TCC_PROGRAM) {
        DATA_TCC_PROGRAM r = get_tcc_program_data(this->gearbox_ptr);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_TCC_PROGRAM, (uint8_t*)&r, sizeof(DATA_TCC_PROGRAM));
    } else if (args[0] == RLI_PRESSURES) {
        DATA_PRESSURES r = get_pressure_data(this->gearbox_ptr);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_PRESSURES, (uint8_t*)&r, sizeof(DATA_PRESSURES));
    } else if (args[0] == RLI_DMA_DUMP) {
        DATA_DMA_BUFFER r = dump_i2s_dma();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_DMA_DUMP, (uint8_t*)&r, sizeof(DATA_DMA_BUFFER));
    } else if (args[0] == RLI_CLUTCH_SPEEDS) {
        ClutchSpeeds r = gearbox->diag_get_clutch_speeds();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_CLUTCH_SPEEDS, (uint8_t*)&r, sizeof(ClutchSpeeds));
    } else if (args[0] == RLI_SHIFTING_ALGO) {
        ShiftAlgoFeedback r = gearbox->algo_feedback;
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_SHIFTING_ALGO, (uint8_t*)&r, sizeof(ShiftAlgoFeedback));
    } else if (args[0] == RLI_TCM_CONFIG) {
        TCM_CORE_CONFIG r = get_tcm_config();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_TCM_CONFIG, (uint8_t*)&r, sizeof(TCM_CORE_CONFIG));
    } else if (args[0] == RLI_EFUSE_CONFIG) {
        TCM_EFUSE_CONFIG ecfg;
        EEPROM::read_efuse_config(&ecfg);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_EFUSE_CONFIG, (uint8_t*)&ecfg, sizeof(TCM_EFUSE_CONFIG));
    } else if (args[0] == RLI_SHIFT_LIVE) {
        SHIFT_LIVE_INFO r = get_shift_live_Data(egs_can_hal, gearbox);
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_SHIFT_LIVE, (uint8_t*)&r, sizeof(SHIFT_LIVE_INFO));
    } else if (args[0] == RLI_FW_HEADER) {
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_FW_HEADER, reinterpret_cast<const uint8_t*>(get_image_header()), sizeof(esp_app_desc_t));
    } else if (args[0] == RLI_COREDUMP_PART_INFO) {
        PARTITION_INFO r = get_coredump_info();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_COREDUMP_PART_INFO, (uint8_t*)&r, sizeof(PARTITION_INFO));
    } else if (args[0] == RLI_CURR_SW_PART_INFO) {
        PARTITION_INFO r = get_current_sw_info();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_CURR_SW_PART_INFO, (uint8_t*)&r, sizeof(PARTITION_INFO));
    } else if (args[0] == RLI_NEXT_SW_PART_INFO) {
        PARTITION_INFO r = get_next_sw_info();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_NEXT_SW_PART_INFO, (uint8_t*)&r, sizeof(PARTITION_INFO));
    } else if (args[0] == RLI_EGS_CAL_LEN) {
        uint16_t len = get_egs_calibration_size();
        uint8_t x[2] = { (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_EGS_CAL_LEN, x, sizeof(uint16_t));
    } else if (args[0] == RLI_SETTINGS_EDIT) {
        // [RLI, MODULE ID]
        if (arg_len != 2) {
            make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        } else {
            uint8_t* buffer;
            uint16_t read_len;
            kwp_result_t res = get_module_settings(args[1], &read_len, &buffer);
            if (NRC_OK == res) {
                make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_SETTINGS_EDIT, buffer, read_len);
                delete[] buffer; // Remember to deallocate!
            } else {
                make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, res);
            }
        }
    }
    else {
        // EGS52 emulation
        if (VEHICLE_CONFIG.egs_can_type == 2) {
            if (args[0] == 0x31) {
                RLI_31_DATA r = get_rli_31(egs_can_hal);
                return make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0x31, (uint8_t*)&r, sizeof(RLI_31_DATA));
            } else if (args[0] == 0x33) {
                RLI_33_DATA r = get_rli_33(egs_can_hal);
                return make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0x33, (uint8_t*)&r, sizeof(RLI_33_DATA));
            } else if (args[0] == 0x32) {
                RLI_32_DATA r = get_rli_32(egs_can_hal);
                return make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0x32, (uint8_t*)&r, sizeof(RLI_32_DATA));
            } else if (args[0] == 0x30) {
                RLI_30_DATA r = get_rli_30(egs_can_hal);
                return make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0x30, (uint8_t*)&r, sizeof(RLI_30_DATA));
            } else if (args[0] == 0xD1) {
                char x[48];
                memset(&x, 0, 48);
                memcpy(&x,&BOARD_CONFIG, sizeof(BOARD_CONFIG));
                return make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0xD1, (uint8_t*)&x, 48);
            }
        }
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
    if (arg_len != 4) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint32_t start = (args[0] << 16) | (args[1] << 8) | args[2]; // Raw address to read from
    uint8_t len = args[3];
    uint32_t end = start + len;
    if (start >= 0x800000 && end <= 0x87D000) {
        // Address is stored in flash
        uint8_t* buffer = (uint8_t*)TCU_HEAP_ALLOC(len);
        if (nullptr != buffer) {
            // Alloc OK
            if (ESP_OK == esp_flash_read(NULL, buffer, 0x349000 + (start-0x800000), len)) {
                make_diag_pos_msg(SID_READ_MEM_BY_ADDRESS, buffer, len);
                TCU_FREE(buffer);
            } else {
                // Read failed
                make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_GENERAL_REJECT);
                TCU_FREE(buffer);
            }
        } else {
            // Alloc Failed
            make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_GENERAL_REJECT);
        }
    } else {
        uint32_t start_ptr = 0;
        // Address is somewhere in memory
        if(end <= 0x2FFFF) { // and start >= 0x000000
            start_ptr = 0x40070000; // SRAM0
        } else if(start >= 0x030000 && end <= 0x04FFFF) {
            start_ptr = 0x400A0000; // SRAM1
        } else if(start >= 0x050000 && end <= 0x071FFF) {
            start_ptr = 0x3FFAE000; // SRAM2
        } else if(start >= 0x100000 && end <= 0x4FFFFF) {
            start_ptr = 0x3F800000; // PSRAM
        }
        if (0 == start_ptr) { // Invalid address range
            make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        } else {
            // Interp as pointer
            make_diag_pos_msg(SID_READ_MEM_BY_ADDRESS, (uint8_t*)start_ptr, len);
        }
    }
}

void Kwp2000_server::process_security_access(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_disable_msg_tx(uint8_t* args, uint16_t arg_len) {
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52) {
        make_diag_neg_msg(SID_DISABLE_NORMAL_MSG_TRANSMISSION, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (arg_len != 1) {
        make_diag_neg_msg(SID_DISABLE_NORMAL_MSG_TRANSMISSION, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    if (!egs_can_hal) {
        make_diag_neg_msg(SID_DISABLE_NORMAL_MSG_TRANSMISSION, NRC_GENERAL_REJECT);
        return;
    }
    bool response = true;
    if (args[0] == 0x01) { response = true; }
    else if (args[0] == 0x02) { response = false; }
    else {
        make_diag_neg_msg(SID_DISABLE_NORMAL_MSG_TRANSMISSION, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    egs_can_hal->disable_normal_msg_transmission();
    if (response) {
        make_diag_pos_msg(SID_DISABLE_NORMAL_MSG_TRANSMISSION, nullptr, 0);
    }
}

void Kwp2000_server::process_enable_msg_tx(uint8_t* args, uint16_t arg_len) {
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52) {
        make_diag_neg_msg(SID_ENABLE_NORMAL_MSG_TRANSMISSION, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (arg_len != 1) {
        make_diag_neg_msg(SID_ENABLE_NORMAL_MSG_TRANSMISSION, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    if (!egs_can_hal) {
        make_diag_neg_msg(SID_ENABLE_NORMAL_MSG_TRANSMISSION, NRC_GENERAL_REJECT);
        return;
    }
    bool response = true;
    if (args[0] == 0x01) { response = true; }
    else if (args[0] == 0x02) { response = false; }
    else {
        make_diag_neg_msg(SID_ENABLE_NORMAL_MSG_TRANSMISSION, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    egs_can_hal->enable_normal_msg_transmission();
    if (response) {
        make_diag_pos_msg(SID_ENABLE_NORMAL_MSG_TRANSMISSION, nullptr, 0);
    }
}
void Kwp2000_server::process_dynamically_define_local_ident(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_write_data_by_ident(uint8_t* args, uint16_t arg_len) {

}

// INPUT OUTPUT CONTROL BY 'LOCAL' IDENTIFIER HANDLER (SID_IOCTL / SidInputOutputControlByLocalIdentifier)
void Kwp2000_server::process_ioctl_by_local_ident(uint8_t* args, uint16_t arg_len) {
    // Session mode check - This service is only supported in Extended, Reprgramming or OEM dependent modes
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52 && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_IOCTL_BY_LOCAL_IDENT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (args[0] == 0x10) { // Mode manipulation

        // NOTE. The device mode responses have been swapped to Big Endian byte order
        //       in order to retain compatibility with Mercedes' diagnostic tool,
        //       as the OEM TCU is a Big Endian processor, where this TCU is Little Endian

        if (arg_len == 2 && args[1] == 0x00) { // Return control back to ECU
            CURRENT_DEVICE_MODE = DEVICE_MODE_NORMAL;
            EEPROM::set_device_mode(DEVICE_MODE_NORMAL);
            uint8_t resp[3] = {0x10, 0x00};
            make_diag_pos_msg(SID_IOCTL_BY_LOCAL_IDENT, resp, 2);
        } else if (arg_len == 2 && args[1] == 0x01) { // Report current device mode
            uint8_t resp[4] = {0x10, 0x01, (uint8_t)((CURRENT_DEVICE_MODE >> 8) & 0xFF), (uint8_t)(CURRENT_DEVICE_MODE & 0xFF)};
            make_diag_pos_msg(SID_IOCTL_BY_LOCAL_IDENT, resp, 4);
        } else if (arg_len == 4 && args[1] == 0x07) { // Just change device mode
            uint16_t mode_req = (args[2] << 8) | args[3];
            CURRENT_DEVICE_MODE = mode_req;
            uint8_t resp[4] = {0x10, 0x07, (uint8_t)((CURRENT_DEVICE_MODE >> 8) & 0xFF), (uint8_t)(CURRENT_DEVICE_MODE & 0xFF)};
            make_diag_pos_msg(SID_IOCTL_BY_LOCAL_IDENT, resp, 4);
        } else if (arg_len == 4 && args[1] == 0x08) { // Change device mode and save to EEPROM!
            uint16_t mode_req = (args[2] << 8) | args[3];
            CURRENT_DEVICE_MODE = mode_req;
            EEPROM::set_device_mode(mode_req);
            uint8_t resp[4] = {0x10, 0x08, (uint8_t)((CURRENT_DEVICE_MODE >> 8) & 0xFF), (uint8_t)(CURRENT_DEVICE_MODE & 0xFF)};
            make_diag_pos_msg(SID_IOCTL_BY_LOCAL_IDENT, resp, 4);
        } else {
            make_diag_neg_msg(SID_IOCTL_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        }
    } else {
        make_diag_neg_msg(SID_IOCTL_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
    }
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

    if (arg_len == 0) {
        make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }

    // EGS emulation
    if (args[0] == ROUTINE_EGS_ID_TCC_SOL_TOGGLE) {
        // Should have 1 more byte
        if (arg_len != 2) {
            make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        } else {
            uint8_t resp[1] = {ROUTINE_EGS_ID_TCC_SOL_TOGGLE};
            if (args[1] == 0x00 || args[1] == 0x01) { // Long term off or short term off
                gearbox_ptr->tcc->diag_toggle_tcc_sol(false);
                make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, resp, 1);
            } else if (args[1] == 0x02) { // Back on
                gearbox_ptr->tcc->diag_toggle_tcc_sol(true);
                make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, resp, 1);
            } else {
                make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            }
        }
        return;
    }

    if (arg_len == 1) {
        if (args[0] == ROUTINE_SOLENOID_TEST) {
            uint16_t voltage = TCUIO::battery_mv();
            uint16_t pll = TCUIO::parking_lock();
            if (
                    gearbox_ptr->sensor_data.engine_rpm == 0 && //Engine off
                    gearbox_ptr->sensor_data.input_rpm == 0 && // Not moving

                    voltage != UINT16_MAX &&
                    voltage > 10000 && // Enough battery voltage
                    pll == 0 // Parking lock off (In D/R)
                ) {
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
                this->flash_handler->on_request_verification(args, arg_len, &this->tx_msg);
                this->send_resp = true;
                return;
            } else {
                make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
            }
        } else if (args[0] == ROUTINE_ADAPTATION_RESET) {
            esp_err_t res = this->gearbox_ptr->shift_adapter->reset();
            if (ESP_OK == res) {
                make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, nullptr, 0);
            } else {
                // Can only fail if adapt manager is nullptr (Not ready)
                make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
            }
        } else if (args[0] == ROUTINE_CALIBRATION_HOT_RELOAD) {
            esp_err_t res = EGSCal::reload_egs_calibration();
            if (ESP_OK == res) {
                make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, nullptr, 0);
            } else {
                make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_GENERAL_REJECT);
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
void Kwp2000_server::process_request_routine_results_by_local_ident(const uint8_t* args, uint16_t arg_len) {
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
        make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (this->flash_handler != nullptr) {
        delete this->flash_handler;
    }
    // Make a new flash handler!
    this->flash_handler = new Flasher(this->can_layer, this->gearbox_ptr);
    this->flash_handler->on_request_download(args, arg_len, &this->tx_msg, !this->diag_on_usb);
    this->send_resp = true;
}

void Kwp2000_server::process_request_upload(uint8_t* args, uint16_t arg_len) {
    // Valid session only
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_REQ_UPLOAD, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (this->flash_handler != nullptr) {
        delete this->flash_handler;
    }
    this->flash_handler = new Flasher(this->can_layer, this->gearbox_ptr);
    this->flash_handler->on_request_upload(args, arg_len, &this->tx_msg, !this->diag_on_usb);
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
        this->flash_handler->on_transfer_data(args, arg_len, &this->tx_msg, !this->diag_on_usb);
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
        this->flash_handler->on_transfer_exit(args, arg_len, &this->tx_msg);
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
        if (args[0] == RLI_MAP_EDITOR) {
            // 0 - RLI
            // 1 - Map ID
            // 2 - CMD
            // 3-4 - Arg len
            // 5..n - Data
            if (arg_len < 5) {
                make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
                return;
            }
            uint8_t map_id = args[1];
            uint8_t cmd = args[2];
            uint16_t map_len_bytes = args[4] << 8 | args[3];
            if (arg_len-5 != map_len_bytes) {
                make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
                return;
            }
            uint8_t ret;
            switch (cmd) {
                case MAP_CMD_WRITE:
                    ret = MapEditor::write_map_data(map_id, map_len_bytes/2, (int16_t*)&args[5]); // len_bytes /2 = sizeof(int16)
                    break;
                case MAP_CMD_UNDO:
                    ret = MapEditor::undo_changes(map_id);
                    break;
                case MAP_CMD_BURN:
                    ret = MapEditor::burn_to_eeprom(map_id);
                    break;
                case MAP_CMD_RESET_TO_FLASH:
                    ret = MapEditor::reset_to_program_default(map_id);
                    break;
                default:
                    ret = NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
                    break;
            }
            if (ret == 0) {
                make_diag_pos_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, nullptr, 0);
            } else {
                make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, ret);
            }
        } else if (args[0] == RLI_TCM_CONFIG) {
            if (arg_len-1 != sizeof(TCM_CORE_CONFIG)) {
                make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            } else {
                // TCM Core config size ok
                TCM_CORE_CONFIG cfg;
                memcpy(&cfg, &args[1], sizeof(TCM_CORE_CONFIG));
                uint8_t res = set_tcm_config(cfg);
                if (res == NRC_OK) {
                    make_diag_pos_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, RLI_TCM_CONFIG, nullptr, 0);
                } else {
                    make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, res);
                }
            }
        } else if (args[0] == RLI_EFUSE_CONFIG) {
            if (arg_len-1 != sizeof(TCM_EFUSE_CONFIG)) {
                make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            } else {
                // TCM Core config size ok
                TCM_EFUSE_CONFIG cfg;
                memcpy(&cfg, &args[1], sizeof(TCM_EFUSE_CONFIG));
                bool res = EEPROM::write_efuse_config(&cfg);
                if (res == ESP_OK) {
                    make_diag_pos_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, RLI_TCM_CONFIG, nullptr, 0);
                } else {
                    make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_GENERAL_REJECT);
                }
            }
        } else if (args[0] == RLI_SETTINGS_EDIT) {
            // [RLI, MODULE ID,...]
            if (arg_len < 3) {
                make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            } else {
                kwp_result_t res = set_module_settings(args[1], arg_len-2, &args[2]);
                if (res == NRC_OK) {
                    make_diag_pos_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, RLI_SETTINGS_EDIT, nullptr, 0);
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
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (arg_len < 4) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint32_t start = (args[0] << 16) | (args[1] << 8) | args[2]; // Raw address to read from
    uint8_t len = args[3];
    if (arg_len-4 != len) { // Length mismatch between message write data, and actual data to write
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint8_t* src = &args[4];
    uint32_t end = start + len;
    if (start >= 0x800000 && end <= 0x87D000) {
        #define SECTOR_SIZE 4096
        int phys_address = 0x349000 + (start-0x800000);
        int sec_start_addr = (phys_address/SECTOR_SIZE)*SECTOR_SIZE;
        int offset_into_start_sector = phys_address - sec_start_addr;
        uint8_t* buffer = (uint8_t*)TCU_HEAP_ALLOC(SECTOR_SIZE);
        esp_flash_read(NULL, buffer, sec_start_addr, SECTOR_SIZE);
        memcpy(&buffer[offset_into_start_sector], src, len);
        esp_flash_erase_region(NULL, sec_start_addr, SECTOR_SIZE);
        if (ESP_OK == esp_flash_write(NULL, buffer, sec_start_addr, SECTOR_SIZE)) {
            make_diag_pos_msg(SID_READ_MEM_BY_ADDRESS, nullptr, 0);
        } else {
            // Read failed
            make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_GENERAL_REJECT);
        }
        delete[] buffer;
    } else {
        uint32_t start_ptr = 0;
        // Address is somewhere in memory
        if(end <= 0x2FFFF) { // and start >= 0x000000
            start_ptr = 0x40070000; // SRAM0
        } else if(start >= 0x030000 && end <= 0x04FFFF) {
            start_ptr = 0x400A0000; // SRAM1
        } else if(start >= 0x050000 && end <= 0x071FFF) {
            start_ptr = 0x3FFAE000; // SRAM2
        } else if(start >= 0x100000 && end <= 0x4FFFFF) {
            start_ptr = 0x3F800000; // PSRAM
        }
        if (0 == start_ptr) { // Invalid address range
            make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        } else {
            // Interp as pointer
            memcpy((void*)start_ptr, (void*)src, len);
            make_diag_pos_msg(SID_READ_MEM_BY_ADDRESS, nullptr, 0);
        }
    }
}

void Kwp2000_server::process_tester_present(const uint8_t* args, uint16_t arg_len) {
    if (arg_len != 1) { // Must only have 1 arg
        make_diag_neg_msg(SID_TESTER_PRESENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    if (args[0] == KWP_CMD_RESPONSE_REQUIRED) {
        make_diag_pos_msg(SID_TESTER_PRESENT, nullptr, 0);
        this->next_tp_time = GET_CLOCK_TIME() + KWP_TP_TIMEOUT_MS;
    } else if (args[1] == KWP_CMD_NO_RESPONSE_REQUIRED) {
        this->next_tp_time = GET_CLOCK_TIME() + KWP_TP_TIMEOUT_MS;
    } else {
        make_diag_neg_msg(SID_TESTER_PRESENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
}
void Kwp2000_server::process_control_dtc_settings(uint8_t* args, uint16_t arg_len) {

}
void Kwp2000_server::process_response_on_event(uint8_t* args, uint16_t arg_len) {

}

void Kwp2000_server::process_shift_mgr_op(uint8_t* args, uint16_t arg_len) {
    if (
        this->session_mode == SESSION_EXTENDED ||
        this->session_mode == SESSION_CUSTOM_UN52
    ) {
        make_diag_neg_msg(SID_SHIFT_MGR_OP, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
        /*
        // Make request message
        // Should be 1 byte argument
        // 0x00 0x00 - Request shift summary
        // 0x01 0xzz - Request shift by ID
        // 0x02 0x00 - Clear shift data
        // 0x03 0x00 - Request current ID of shift index (Can be used to see if new data is avaliable)

        if (arg_len != 2) {
            make_diag_neg_msg(SID_SHIFT_MGR_OP, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
        }
        if (gearbox_ptr->shifting) {
            // Cannot read WHILST shifting
            make_diag_neg_msg(SID_SHIFT_MGR_OP, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
            return;
        }
        ShiftReportNvsGroup grp = this->gearbox_ptr->shift_reporter->diag_get_nvs_group_ptr();
        if (args[0] == 0x00) {
            // Each report needs 4 bytes, [ID, TAR_CUR_GEAR, ATF, ATF]
            uint8_t resp[4*MAX_REPORTS];
            memset(resp, 0x00, sizeof(resp));
            for (uint8_t i = 0; i < MAX_REPORTS; i++) {
                uint8_t* ptr = &resp[i*4];
                ptr[0] = i;
                ptr[1] = grp.reports[i].targ_curr;
                ptr[2] = grp.reports[i].atf_temp_c >> 8;
                ptr[3] = grp.reports[i].atf_temp_c & 0xFF;
            }
            make_diag_pos_msg(SID_SHIFT_MGR_OP, resp, sizeof(resp));
            return;
        } else if (args[0] == 0x01) {
            if (args[1] >= MAX_REPORTS) {
                make_diag_neg_msg(SID_SHIFT_MGR_OP, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
                return;
            }
            // OK, get the data
            make_diag_pos_msg(SID_SHIFT_MGR_OP, (const uint8_t*)&grp.reports[args[1]], sizeof(ShiftReport));
        } else if (args[0] == 0x02) {
            // TODO clear shift data
            make_diag_pos_msg(SID_SHIFT_MGR_OP, nullptr, 0);
        } else if (args[0] == 0x03) {
            make_diag_pos_msg(SID_SHIFT_MGR_OP, &grp.index, 1);
        } else {
            make_diag_neg_msg(SID_SHIFT_MGR_OP, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        }
        */
    } else {
        make_diag_neg_msg(SID_SHIFT_MGR_OP, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
    }
}

typedef struct {
    uint8_t lid;
    int16_t atf_temp;
    uint16_t off_test[6];
    SolenoidTestReading on_readings[6];
} __attribute__((packed)) SolRtRes;

static_assert(sizeof(SolRtRes) == 1+2+6*6);

void Kwp2000_server::run_solenoid_test() {
    vTaskDelay(50);
    this->routine_results_len = 1 + sizeof(SolRtRes); // ATF temp (2 byte), (current off, current on, vbatt) (x6);
    memset(this->routine_result, 0, this->routine_results_len);
    this->routine_result[0] = this->routine_id;
    // Routine results format

    SolRtRes res{};
    res.lid = this->routine_id;
    int16_t temp = TCUIO::atf_temperature();
    uint8_t pll = TCUIO::parking_lock();
    if (pll != 0 || INT16_MAX == temp) {
        return; // Cannot function unless PLL is off
    }
    res.atf_temp = temp;
    if (nullptr != this->gearbox_ptr) {
        this->gearbox_ptr->diag_inhibit_control();
    }
    Solenoids::notify_diag_test_start();

    PwmSolenoid* order[6] = {sol_mpc, sol_spc, sol_tcc, sol_y3, sol_y4, sol_y5};
    for (uint8_t i = 0; i < 6; i++) {
        uint16_t current = order[i]->get_current();
        // place in result
        res.off_test[i] = current;
    }
    // Now do on tests
    for (uint8_t i = 0; i < 6; i++) {
        order[i]->pre_current_test();
        SolenoidTestReading t = order[i]->get_full_on_current_reading();
        order[i]->post_current_test();
        // place in result
        res.on_readings[i] = t;
    }
    this->routine_running = false;
    Solenoids::notify_diag_test_end();
    if (nullptr != this->gearbox_ptr) {
        this->gearbox_ptr->diag_regain_control();
    }
    memcpy(this->routine_result, &res, sizeof(SolRtRes));
    vTaskDelete(nullptr);
}