#include "kwp2000.h"
#include <algorithm>
#include <esp_ota_ops.h>
#include <string>
#include <time.h>
#include "diag_data.h"
#include "egs_emulation.h"
#include "kwp2000_logic.h"
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

    uint8_t day = (uint8_t)(((date[0] - '0') * 10) + (date[1] - '0'));
    uint8_t year = (uint8_t)(((date[9] - '0') * 10) + (date[10] - '0'));
    struct tm time;
    memset(&time, 0, sizeof(time));
    char timebuf[8];
    memset(timebuf, 0, sizeof(timebuf));
    time.tm_mday = day;
    time.tm_year = 100 + year;
    time.tm_mon = month - 1;
    mktime(&time);
    // %W is the week number (00-53). The '0' flag is not portable, so pad here
    // instead. strftime returns 0 (and leaves the buffer unspecified) on error,
    // which is why the buffer is zeroed and the result length is checked.
    size_t written = strftime(timebuf, sizeof(timebuf), "%W", &time);
    const uint8_t week = (2u == written)
        ? (uint8_t)(((timebuf[0] - '0') * 10) + (timebuf[1] - '0'))
        : ((1u == written) ? (uint8_t)(timebuf[0] - '0') : 0u);
    ECU_Date out{};
    out.day = day;
    out.month = month;
    out.year = year;
    out.week = week;
    return out;
}

Kwp2000_server::Kwp2000_server(EgsBaseCan* can_layer, Gearbox* gearbox) {
    // Init SPIRAM (We will need this!)
    this->next_tp_time = 0;
    this->session_mode = SESSION_DEFAULT;
    this->state_mutex = portMUX_INITIALIZER_UNLOCKED;
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
    free(this->running_routine_args);
    this->running_routine_args = nullptr;
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
    this->running_routine_args = nullptr; // Never leave the pointer dangling
    if (nullptr == src || 0 == arg_len) {
        return 0;
    }
    this->running_routine_args = static_cast<uint8_t*>(malloc(arg_len));
    if (nullptr == this->running_routine_args) {
        return -1;
    }
    memcpy(this->running_routine_args, src, arg_len);
    return arg_len;
}

void Kwp2000_server::start_response_timer(uint8_t sid) {
    portENTER_CRITICAL(&this->state_mutex);
    this->response_pending_sid = sid;
    this->cmd_recv_time = GET_CLOCK_TIME();
    this->response_pending = true;
    portEXIT_CRITICAL(&this->state_mutex);
}

void Kwp2000_server::end_response_timer() {
    portENTER_CRITICAL(&this->state_mutex);
    this->response_pending = false;
    portEXIT_CRITICAL(&this->state_mutex);
}

void Kwp2000_server::response_timer_loop() {
    uint8_t buf[3] = {0x7F, 0x00, NRC_RESPONSE_PENDING};
    while(1) {
        bool pending = false;
        uint8_t pending_sid = 0;
        uint32_t pending_since = 0;
        bool route_usb = false;
        portENTER_CRITICAL(&this->state_mutex);
        pending = this->response_pending;
        pending_sid = this->response_pending_sid;
        pending_since = this->cmd_recv_time;
        route_usb = this->diag_on_usb;
        portEXIT_CRITICAL(&this->state_mutex);

        if (pending && (GET_CLOCK_TIME() - pending_since) > KWP_RESPONSEPENDING_INTERVAL) {
            buf[1] = pending_sid;
            // Send 0x78 (Response pending)
            if (route_usb) {
                this->usb_diag_endpoint->send_data(KWP_ECU_TX_ID, buf, 3);
            } else {
                this->can_endpoint->send_data(KWP_ECU_TX_ID, buf, 3);
            }
            portENTER_CRITICAL(&this->state_mutex);
            this->cmd_recv_time = GET_CLOCK_TIME();
            portEXIT_CRITICAL(&this->state_mutex);
        }
        vTaskDelay(250/portTICK_PERIOD_MS);
    }
}

// cppcheck-suppress functionConst
bool Kwp2000_server::dispatch_sid(uint8_t sid, uint8_t* args, uint16_t arg_len) {
    struct ConstSidRoute {
        uint8_t sid;
        // cppcheck-suppress unusedStructMember
        void (Kwp2000_server::*handler)(const uint8_t*, uint16_t);
    };
    struct MutableSidRoute {
        uint8_t sid;
        // cppcheck-suppress unusedStructMember
        void (Kwp2000_server::*handler)(uint8_t*, uint16_t);
    };

    static const ConstSidRoute const_routes[] = {
        {SID_START_DIAGNOSTIC_SESSION, &Kwp2000_server::process_start_diag_session},
        {SID_ECU_RESET, &Kwp2000_server::process_ecu_reset},
        {SID_READ_MEM_BY_ADDRESS, &Kwp2000_server::process_read_mem_address},
        {SID_READ_MEM_BY_ADDRESS_EXT, &Kwp2000_server::process_read_mem_address_ext},
        {SID_READ_ECU_IDENT, &Kwp2000_server::process_read_ecu_ident},
        {SID_TESTER_PRESENT, &Kwp2000_server::process_tester_present},
        {SID_REQUEST_ROUTINE_RESULTS_BY_LOCAL_IDENT, &Kwp2000_server::process_request_routine_results_by_local_ident},
        {SID_IOCTL_BY_LOCAL_IDENT, &Kwp2000_server::process_ioctl_by_local_ident},
        {SID_ENABLE_NORMAL_MSG_TRANSMISSION, &Kwp2000_server::process_enable_msg_tx},
        {SID_DISABLE_NORMAL_MSG_TRANSMISSION, &Kwp2000_server::process_disable_msg_tx},
    };

    auto const_it = std::find_if(
        std::begin(const_routes),
        std::end(const_routes),
        [sid](const ConstSidRoute& route) { return route.sid == sid; }
    );
    if (const_it != std::end(const_routes)) {
        (this->*const_it->handler)(args, arg_len);
        return true;
    }

    static const MutableSidRoute mutable_routes[] = {
        {SID_READ_DATA_LOCAL_IDENT, &Kwp2000_server::process_read_data_local_ident},
        {SID_WRITE_DATA_BY_LOCAL_IDENT, &Kwp2000_server::process_write_data_by_local_ident},
        {SID_WRITE_MEM_BY_ADDRESS, &Kwp2000_server::process_write_mem_by_address},
        {SID_START_ROUTINE_BY_LOCAL_IDENT, &Kwp2000_server::process_start_routine_by_local_ident},
        {SID_REQ_UPLOAD, &Kwp2000_server::process_request_upload},
        {SID_REQ_DOWNLOAD, &Kwp2000_server::process_request_download},
        {SID_TRANSFER_DATA, &Kwp2000_server::process_transfer_data},
        {SID_TRANSFER_EXIT, &Kwp2000_server::process_transfer_exit},
        {SID_SHIFT_MGR_OP, &Kwp2000_server::process_shift_mgr_op},
    };

    auto mutable_it = std::find_if(
        std::begin(mutable_routes),
        std::end(mutable_routes),
        [sid](const MutableSidRoute& route) { return route.sid == sid; }
    );
    if (mutable_it != std::end(mutable_routes)) {
        (this->*mutable_it->handler)(args, arg_len);
        return true;
    }

    return false;
}

void Kwp2000_server::server_loop() {
    this->send_resp = false;
    while(1) {
        PerfMon::update_sample();
        uint32_t timestamp = GET_CLOCK_TIME();
        bool read_msg = false;
        if (this->usb_diag_endpoint->init_state() == ESP_OK && this->usb_diag_endpoint->read_data(&this->rx_msg)) {
            portENTER_CRITICAL(&this->state_mutex);
            this->diag_on_usb = true;
            portEXIT_CRITICAL(&this->state_mutex);
            read_msg = true;
        } else if (this->can_endpoint->init_state() == ESP_OK && this->can_endpoint->read_data(&this->rx_msg)) {
            portENTER_CRITICAL(&this->state_mutex);
            this->diag_on_usb = false;
            portEXIT_CRITICAL(&this->state_mutex);
            read_msg = true;
        }
        if (read_msg) {
            this->next_tp_time = timestamp + KWP_TP_TIMEOUT_MS;
            if (this->rx_msg.data_size == 0) {
                continue; // Huh?
            }

            // New message! process it
            uint8_t* args_ptr = &rx_msg.data[1];
            uint16_t args_size = rx_msg.data_size - 1;
            start_response_timer(rx_msg.data[0]);
            if (!this->dispatch_sid(rx_msg.data[0], args_ptr, args_size)) {
                ESP_LOG_LEVEL(ESP_LOG_WARN, "KWP_HANDLE_REQ", "Requested SID %02X is not supported, full msg was:", rx_msg.data[0]);
                ESP_LOG_BUFFER_HEX_LEVEL("KWP_HANDLE_REQ", rx_msg.data, rx_msg.data_size, ESP_LOG_WARN);
                make_diag_neg_msg(rx_msg.data[0], NRC_SERVICE_NOT_SUPPORTED);
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
            // Wrap-safe comparison. GET_CLOCK_TIME() rolls over every ~49 days
            && (int32_t)(timestamp - this->next_tp_time) > 0
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
            vTaskDelay(20);
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
            this->next_tp_time = GET_CLOCK_TIME() + KWP_TP_TIMEOUT_MS;
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
            if (kwp_is_valid_ecu_reset_subfn(args, arg_len)) {
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
    if (!kwp_has_valid_read_data_local_ident_header(args, arg_len)) {
        make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    if (args[0] >= 0x80 && args[0] <= 0x9F) { // ECU Ident
        this->process_read_ecu_ident(args, arg_len); // Modify the SID byte in pos/neg response to be SID_READ_DATA_LOCAL_IDENT
        if (this->tx_msg.data[0] == 0x7F) {
            this->tx_msg.data[1] = SID_READ_DATA_LOCAL_IDENT;
        } else {
            this->tx_msg.data[0] = SID_READ_DATA_LOCAL_IDENT + 0x40;
        }
    } else if (args[0] == 0xE1) { // ECU Serial number
        uint8_t mac[6] = {0};
        esp_efuse_mac_get_default(mac);
        char resp[13];
        int len = snprintf(resp, sizeof(resp), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        if (len < 0) {
            make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_GENERAL_REJECT);
            return;
        }
        if (len > 12) {
            len = 12;
        }
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0xE1, reinterpret_cast<const uint8_t*>(resp), (uint16_t)len);
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
        uint16_t map_len_bytes = (uint16_t)(((uint16_t)args[3] << 8) | (uint16_t)args[4]);
        if ((arg_len - 5) != map_len_bytes) {
            make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
        }
        uint8_t ret;
        uint8_t* buffer = nullptr;
        uint16_t read_bytes_size = 0;
        if (cmd == MAP_CMD_READ || cmd == MAP_CMD_READ_DEFAULT || cmd == MAP_CMD_READ_EEPROM) {
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
        } else if (cmd == MAP_CMD_GET_LOOKUP_VALS && map_len_bytes == 0) {
            ret = MapEditor::read_map_lookup_cache(map_id, &read_bytes_size, &buffer);
        } else {
            ret = NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
        }
        if (ret == 0) { // OK
            if (read_bytes_size > DIAG_CAN_MAX_SIZE - 2) {
                TCU_FREE(buffer); // DELETE MapEditor allocation
                make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_GENERAL_REJECT);
                return;
            }
            uint8_t* buf = static_cast<uint8_t*>(TCU_HEAP_ALLOC(2 + read_bytes_size));
            if (buf == nullptr) {
                TCU_FREE(buffer); // DELETE MapEditor allocation
                make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_GENERAL_REJECT);
                return;
            }
            buf[0] = read_bytes_size & 0xFF;
            buf[1] = read_bytes_size >> 8;
            memcpy(&buf[2], buffer, read_bytes_size);
            make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, buf, 2 + read_bytes_size);
            TCU_FREE(buf);
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
    } else if (args[0] == RLI_TCU_TIME) {
        uint32_t now = GET_CLOCK_TIME();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_TCU_TIME, (uint8_t*)&now, sizeof(now));
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
    } else if (args[0] == RLI_EMBED_FILE_INFO) { 
        PARTITION_INFO r = get_embeded_file_info();
        make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, RLI_EMBED_FILE_INFO, (uint8_t*)&r, sizeof(PARTITION_INFO));
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
                TCU_FREE(buffer); // Remember to deallocate! (Allocated with TCU_HEAP_ALLOC)
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
                memcpy(&x, &BOARD_CONFIG, sizeof(BOARD_CONFIG));
                return make_diag_pos_msg(SID_READ_DATA_LOCAL_IDENT, 0xD1, (uint8_t*)&x, 48);
            }
        }
        make_diag_neg_msg(SID_READ_DATA_LOCAL_IDENT, NRC_REQUEST_OUT_OF_RANGE);
    }
    
}

void Kwp2000_server::process_read_data_ident(const uint8_t* args, uint16_t arg_len) {
    (void)args;
    (void)arg_len;

}

namespace {

/**
 * @brief A logical (KWP) address window, and the CPU address range it maps onto.
 *
 * The offset into the window must be preserved when resolving to a pointer,
 * otherwise every access in the window lands on the first byte of the region.
 */
struct KwpMemWindow {
    uint32_t logical_start;
    uint32_t logical_end; // Inclusive
    uint32_t cpu_base;
    bool word_access_only; // IRAM only supports aligned 32bit access on Xtensa
};

const KwpMemWindow KWP_MEM_WINDOWS[] = {
    {0x000000u, 0x02FFFFu, 0x40070000u, true },  // SRAM0 (IRAM)
    {0x030000u, 0x04FFFFu, 0x400A0000u, true },  // SRAM1 (IRAM)
    {0x050000u, 0x071FFFu, 0x3FFAE000u, false},  // SRAM2 (DRAM)
    {0x100000u, 0x4FFFFFu, 0x3F800000u, false},  // PSRAM
};

/**
 * @brief Resolves a logical address range to the window that fully contains it.
 * @return nullptr if the range is empty, wraps, or straddles a window boundary
 */
const KwpMemWindow* kwp_find_mem_window(uint32_t start, uint32_t len) {
    const KwpMemWindow* ret = nullptr;
    if (0u != len && (start + len) > start) { // Non empty and no wrap
        const uint32_t end = start + len - 1u;
        auto it = std::find_if(
            std::begin(KWP_MEM_WINDOWS),
            std::end(KWP_MEM_WINDOWS),
            [start, end](const KwpMemWindow& window) {
                return start >= window.logical_start && end <= window.logical_end;
            }
        );
        if (it != std::end(KWP_MEM_WINDOWS)) {
            ret = &(*it);
        }
    }
    return ret;
}

/**
 * @brief CPU address ranges that may be read via SID_READ_MEM_BY_ADDRESS_EXT.
 *        Anything outside these traps the CPU, so it must be rejected.
 */
struct KwpCpuWindow {
    uint32_t start;
    uint32_t end; // Inclusive
};

const KwpCpuWindow KWP_CPU_READ_WINDOWS[] = {
    {0x3F400000u, 0x3F7FFFFFu}, // DROM (memory mapped flash)
    {0x3F800000u, 0x3FBFFFFFu}, // PSRAM
    {0x3FF80000u, 0x3FFFFFFFu}, // Internal DRAM
    {0x40000000u, 0x400C1FFFu}, // Internal ROM + IRAM
    {0x400D0000u, 0x403FFFFFu}, // IROM (memory mapped flash)
};

bool kwp_cpu_range_readable(uint32_t start, uint32_t len) {
    bool ret = false;
    if (0u != len && (start + len) > start) { // Non empty and no wrap
        const uint32_t end = start + len - 1u;
        ret = std::any_of(
            std::begin(KWP_CPU_READ_WINDOWS),
            std::end(KWP_CPU_READ_WINDOWS),
            [start, end](const KwpCpuWindow& window) {
                return start >= window.start && end <= window.end;
            }
        );
    }
    return ret;
}

/**
 * @brief Byte-wise read that is safe for IRAM, which faults on unaligned or
 *        sub-word access. Reads the containing word and extracts the byte.
 */
void kwp_word_safe_read(uint8_t* dest, uint32_t src_addr, uint32_t len) {
    for (uint32_t i = 0u; i < len; i++) {
        uint32_t addr = src_addr + i;
        const uint32_t* word_ptr = reinterpret_cast<const uint32_t*>(addr & ~0x3u);
        dest[i] = (uint8_t)((*word_ptr >> (8u * (addr & 0x3u))) & 0xFFu);
    }
}

} // namespace

void Kwp2000_server::process_read_mem_address(const uint8_t* args, uint16_t arg_len) {
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (arg_len != 4) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint32_t start = kwp_read_u24_be(args); // Raw address to read from
    uint8_t len = args[3];
    if (0 == len) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
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
        // Address is somewhere in memory
        const KwpMemWindow* window = kwp_find_mem_window(start, len);
        if (nullptr == window) { // Invalid address range
            make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        } else {
            // Resolve to a pointer, keeping the offset into the window
            uint32_t addr = window->cpu_base + (start - window->logical_start);
            uint8_t* buffer = (uint8_t*)TCU_HEAP_ALLOC(len);
            if (nullptr == buffer) {
                make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_GENERAL_REJECT);
            } else {
                kwp_word_safe_read(buffer, addr, len);
                make_diag_pos_msg(SID_READ_MEM_BY_ADDRESS, buffer, len);
                TCU_FREE(buffer);
            }
        }
    }
}

void Kwp2000_server::process_read_mem_address_ext(const uint8_t* args, uint16_t arg_len) {
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    // 1 byte for size, 4 bytes for addr
    if (arg_len != 5) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint32_t start = kwp_read_u32_be(args); // Raw address to read from
    //ESP_LOGI("RME","%08X\n", (unsigned int)start);
    //vTaskDelay(40);
    uint8_t len = args[4];
    // The address comes straight off the wire, so it has to be checked against
    // the mappable windows. Reading outside them faults the CPU.
    if (!kwp_cpu_range_readable(start, len)) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS_EXT, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    uint8_t* buffer = (uint8_t*)TCU_HEAP_ALLOC(len);
    if (buffer == nullptr) {
        make_diag_neg_msg(SID_READ_MEM_BY_ADDRESS_EXT, NRC_GENERAL_REJECT);
        return;
    }
    kwp_word_safe_read(buffer, start, len);
    make_diag_pos_msg(SID_READ_MEM_BY_ADDRESS_EXT, buffer, len);
    TCU_FREE(buffer);
}

void Kwp2000_server::process_security_access(const uint8_t* args, uint16_t arg_len) {
    (void)args;
    (void)arg_len;

}
void Kwp2000_server::process_disable_msg_tx(const uint8_t* args, uint16_t arg_len) {
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

void Kwp2000_server::process_enable_msg_tx(const uint8_t* args, uint16_t arg_len) {
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
void Kwp2000_server::process_dynamically_define_local_ident(const uint8_t* args, uint16_t arg_len) {
    (void)args;
    (void)arg_len;

}
void Kwp2000_server::process_write_data_by_ident(const uint8_t* args, uint16_t arg_len) {
    (void)args;
    (void)arg_len;

}

// INPUT OUTPUT CONTROL BY 'LOCAL' IDENTIFIER HANDLER (SID_IOCTL / SidInputOutputControlByLocalIdentifier)
void Kwp2000_server::process_ioctl_by_local_ident(const uint8_t* args, uint16_t arg_len) {
    // Session mode check - This service is only supported in Extended, Reprgramming or OEM dependent modes
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52 && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_IOCTL_BY_LOCAL_IDENT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (!kwp_has_arg0(args, arg_len)) {
        make_diag_neg_msg(SID_IOCTL_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    if (args[0] == 0x10) { // Mode manipulation

        // NOTE. The device mode responses have been swapped to Big Endian byte order
        //       in order to retain compatibility with Mercedes' diagnostic tool,
        //       as the OEM TCU is a Big Endian processor, where this TCU is Little Endian

        if (arg_len == 2 && args[1] == 0x00) { // Return control back to ECU
            CURRENT_DEVICE_MODE = DEVICE_MODE_NORMAL;
            EEPROM::set_device_mode(DEVICE_MODE_NORMAL);
            uint8_t resp[2] = {0x10, 0x00};
            make_diag_pos_msg(SID_IOCTL_BY_LOCAL_IDENT, resp, 2);
        } else if (arg_len == 2 && args[1] == 0x01) { // Report current device mode
            uint8_t resp[4] = {0x10, 0x01, (uint8_t)((CURRENT_DEVICE_MODE >> 8) & 0xFF), (uint8_t)(CURRENT_DEVICE_MODE & 0xFF)};
            make_diag_pos_msg(SID_IOCTL_BY_LOCAL_IDENT, resp, 4);
        } else if (arg_len == 4 && (args[1] == 0x07 || args[1] == 0x08)) { // Change device mode (0x08 also persists it)
            uint16_t mode_req = (args[2] << 8) | args[3];
            // Reject unknown bits. Mode 0x08 persists this to NVS, so an
            // unrecognised value would survive a reboot.
            if (0 != (mode_req & (uint16_t)~DEVICE_MODE_VALID_MASK)) {
                make_diag_neg_msg(SID_IOCTL_BY_LOCAL_IDENT, NRC_REQUEST_OUT_OF_RANGE);
                return;
            }
            CURRENT_DEVICE_MODE = mode_req;
            if (args[1] == 0x08) {
                EEPROM::set_device_mode(mode_req);
            }
            uint8_t resp[4] = {0x10, args[1], (uint8_t)((CURRENT_DEVICE_MODE >> 8) & 0xFF), (uint8_t)(CURRENT_DEVICE_MODE & 0xFF)};
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
    bool running_now = false;
    portENTER_CRITICAL(&this->state_mutex);
    running_now = this->routine_running;
    portEXIT_CRITICAL(&this->state_mutex);
    if (running_now) {
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
                portENTER_CRITICAL(&this->state_mutex);
                this->routine_running = true;
                portEXIT_CRITICAL(&this->state_mutex);
                this->routine_id = ROUTINE_SOLENOID_TEST;
                if (xTaskCreate(Kwp2000_server::launch_solenoid_test, "RT_SOL_TEST", 2048, this, 5, &this->routine_task) != pdPASS) {
                    portENTER_CRITICAL(&this->state_mutex);
                    this->routine_running = false;
                    portEXIT_CRITICAL(&this->state_mutex);
                    make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_GENERAL_REJECT);
                    return;
                }
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
                // The pressure manager holds its own aligned copy of the PCS
                // axes, so it has to be told to re-read them.
                if (nullptr != pressure_manager) {
                    pressure_manager->reload_calibration_maps();
                }
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
void Kwp2000_server::process_stop_routine_by_local_ident(const uint8_t* args, uint16_t arg_len) {
    (void)args;
    (void)arg_len;
    
}
void Kwp2000_server::process_request_routine_results_by_local_ident(const uint8_t* args, uint16_t arg_len) {
    if (this->session_mode != SESSION_EXTENDED && this->session_mode != SESSION_CUSTOM_UN52  && this->session_mode != SESSION_REPROGRAMMING) {
        make_diag_neg_msg(SID_REQUEST_ROUTINE_RESULTS_BY_LOCAL_IDENT, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    bool running_now = false;
    portENTER_CRITICAL(&this->state_mutex);
    running_now = this->routine_running;
    portEXIT_CRITICAL(&this->state_mutex);
    if (running_now) {
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
        if (!kwp_has_arg0(args, arg_len)) {
            make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
        }
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
            uint16_t map_len_bytes = (uint16_t)(((uint16_t)args[4] << 8) | (uint16_t)args[3]);
            if ((arg_len - 5) != map_len_bytes) {
                make_diag_neg_msg(SID_WRITE_DATA_BY_LOCAL_IDENT, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
                return;
            }
            uint8_t ret;
            switch (cmd) {
                case MAP_CMD_WRITE:
                    ret = MapEditor::write_map_data(map_id, (uint16_t)(map_len_bytes / 2u), (int16_t*)&args[5]); // len_bytes / 2 = sizeof(int16)
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
            if ((arg_len - 1) != sizeof(TCM_CORE_CONFIG)) {
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
            if ((arg_len - 1) != sizeof(TCM_EFUSE_CONFIG)) {
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
                kwp_result_t res = set_module_settings(args[1], (uint16_t)(arg_len - 2), &args[2]);
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
        make_diag_neg_msg(SID_WRITE_MEM_BY_ADDRESS, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION);
        return;
    }
    if (arg_len < 4) {
        make_diag_neg_msg(SID_WRITE_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint32_t start = kwp_read_u24_be(args); // Raw address to write to
    uint8_t len = args[3];
    if (0 == len || (arg_len - 4) != len) { // Length mismatch between message write data, and actual data to write
        make_diag_neg_msg(SID_WRITE_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint8_t* src = &args[4];
    uint32_t end = start + len;
    if (start >= 0x800000 && end <= 0x87D000) {
        #define SECTOR_SIZE (4096)
        int phys_address = 0x349000 + (start - 0x800000);
        int sec_start_addr = (phys_address / SECTOR_SIZE) * SECTOR_SIZE;
        int offset_into_start_sector = phys_address - sec_start_addr;
        if ((offset_into_start_sector + len) > SECTOR_SIZE) {
            make_diag_neg_msg(SID_WRITE_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
        }
        uint8_t* buffer = (uint8_t*)TCU_HEAP_ALLOC(SECTOR_SIZE);
        if (buffer == nullptr) {
            make_diag_neg_msg(SID_WRITE_MEM_BY_ADDRESS, NRC_GENERAL_REJECT);
            return;
        }
        // Read/modify/erase/write the whole sector. Bail out before erasing if
        // the read failed, otherwise we would blank the sector with junk.
        esp_err_t res = esp_flash_read(NULL, buffer, sec_start_addr, SECTOR_SIZE);
        if (ESP_OK == res) {
            memcpy(&buffer[offset_into_start_sector], src, len);
            res = esp_flash_erase_region(NULL, sec_start_addr, SECTOR_SIZE);
        }
        if (ESP_OK == res) {
            res = esp_flash_write(NULL, buffer, sec_start_addr, SECTOR_SIZE);
        }
        if (ESP_OK == res) {
            make_diag_pos_msg(SID_WRITE_MEM_BY_ADDRESS, nullptr, 0);
        } else {
            make_diag_neg_msg(SID_WRITE_MEM_BY_ADDRESS, NRC_GENERAL_REJECT);
        }
        TCU_FREE(buffer);
    } else {
        // Address is somewhere in memory
        const KwpMemWindow* window = kwp_find_mem_window(start, len);
        if (nullptr == window) { // Invalid address range
            make_diag_neg_msg(SID_WRITE_MEM_BY_ADDRESS, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        } else if (window->word_access_only) {
            // Byte-wise writes to IRAM fault, and a read/modify/write of live
            // instruction memory is not something we want to do from diag.
            make_diag_neg_msg(SID_WRITE_MEM_BY_ADDRESS, NRC_REQUEST_OUT_OF_RANGE);
        } else {
            // Resolve to a pointer, keeping the offset into the window
            uint32_t addr = window->cpu_base + (start - window->logical_start);
            memcpy((void*)addr, (const void*)src, len);
            make_diag_pos_msg(SID_WRITE_MEM_BY_ADDRESS, nullptr, 0);
        }
    }
}

void Kwp2000_server::process_tester_present(const uint8_t* args, uint16_t arg_len) {
    KwpTesterPresentSubfn subfn = kwp_parse_tester_present_subfn(args, arg_len);
    if (subfn == KwpTesterPresentSubfn::ResponseRequired) {
        make_diag_pos_msg(SID_TESTER_PRESENT, nullptr, 0);
        this->next_tp_time = GET_CLOCK_TIME() + KWP_TP_TIMEOUT_MS;
    } else if (subfn == KwpTesterPresentSubfn::NoResponseRequired) {
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

static_assert(sizeof(SolRtRes) == (1 + 2 + (6 * 6)));

void Kwp2000_server::run_solenoid_test() {
    bool inhibited_control = false;
    bool suspended_solenoids = false;
    vTaskDelay(50);
    this->routine_results_len = 1 + sizeof(SolRtRes); // ATF temp (2 byte), (current off, current on, vbatt) (x6);
    memset(this->routine_result, 0, this->routine_results_len);
    this->routine_result[0] = this->routine_id;
    // Routine results format

    SolRtRes res{};
    res.lid = this->routine_id;
    PwmSolenoid* order[6] = {sol_mpc, sol_spc, sol_tcc, sol_y3, sol_y4, sol_y5};
    temp_c_t temp = TCUIO::atf_temperature();
    uint8_t pll = TCUIO::parking_lock();
    if (pll != 0 || !Temp::is_valid(temp)) {
        goto cleanup;
    }
    // SolRtRes is a packed wire struct, so store raw Celsius.
    res.atf_temp = Temp::celsius_i16(temp);
    if (nullptr != this->gearbox_ptr) {
        this->gearbox_ptr->diag_inhibit_control();
        inhibited_control = true;
    }
    Solenoids::notify_diag_test_start();
    suspended_solenoids = true;
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
cleanup:
    if (suspended_solenoids) {
        Solenoids::notify_diag_test_end();
    }
    if (inhibited_control && nullptr != this->gearbox_ptr) {
        this->gearbox_ptr->diag_regain_control();
    }
    portENTER_CRITICAL(&this->state_mutex);
    this->routine_running = false;
    portEXIT_CRITICAL(&this->state_mutex);
    memcpy(this->routine_result, &res, sizeof(SolRtRes));
    vTaskDelete(nullptr);
}
