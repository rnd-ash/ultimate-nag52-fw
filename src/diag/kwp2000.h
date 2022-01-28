

#ifndef _KWP_H__
#define _KWP_H__

#include "endpoint.h"
#include <stdint.h>
#include "kwp2000_defines.h"
#include "gearbox.h"
#include "canbus/can_hal.h"
#include "gearbox_config.h"

// Ident data

#ifdef EGS53_MODE
    #define SUPPLIER_ID 0x08 // Simens
    #define DIAG_VARIANT_CODE 0x0353 // DiagVersion53_EGS53
#endif

#ifdef EGS52_MODE
    #define SUPPLIER_ID 0x08 // Simens
    #define DIAG_VARIANT_CODE 0x0251 // DiagVersion51_EGS52
#endif

class Kwp2000_server {
    public:
        Kwp2000_server(AbstractCan* can_layer, Gearbox* gearbox);

        static void start_kwp_server(void *_this) {
            static_cast<Kwp2000_server*>(_this)->server_loop();
        }

    private:
        [[noreturn]]
        void server_loop();
        uint8_t session_mode;
        uint64_t next_tp_time;
        DiagMessage tx_msg;
        DiagMessage rx_msg;
        UsbEndpoint* usb_diag_endpoint;
        CanEndpoint* can_endpoint;

        bool send_resp;
        bool reboot_pending;

        void process_start_diag_session(uint8_t* args, uint16_t arg_len);
        void process_ecu_reset(uint8_t* args, uint16_t arg_len);
        void process_clear_diag_info(uint8_t* args, uint16_t arg_len);
        void process_read_status_of_dtcs(uint8_t* args, uint16_t arg_len);
        void process_read_ecu_ident(uint8_t* args, uint16_t arg_len);
        void process_read_data_local_ident(uint8_t* args, uint16_t arg_len);
        void process_read_data_ident(uint8_t* args, uint16_t arg_len);
        void process_read_mem_address(uint8_t* args, uint16_t arg_len);
        void process_security_access(uint8_t* args, uint16_t arg_len);
        void process_disable_msg_tx(uint8_t* args, uint16_t arg_len);
        void process_enable_msg_tx(uint8_t* args, uint16_t arg_len);
        void process_dynamically_define_local_ident(uint8_t* args, uint16_t arg_len);
        void process_write_data_by_ident(uint8_t* args, uint16_t arg_len);
        void process_ioctl_by_local_ident(uint8_t* args, uint16_t arg_len);
        void process_start_routine_by_local_ident(uint8_t* args, uint16_t arg_len);
        void process_stop_routine_by_local_ident(uint8_t* args, uint16_t arg_len);
        void process_request_routine_resutls_by_local_ident(uint8_t* args, uint16_t arg_len);
        void process_request_download(uint8_t* args, uint16_t arg_len);
        void process_request_upload(uint8_t* args, uint16_t arg_len);
        void process_transfer_data(uint8_t* args, uint16_t arg_len);
        void process_transfer_exit(uint8_t* args, uint16_t arg_len);
        void process_write_data_by_local_ident(uint8_t* args, uint16_t arg_len);
        void process_write_mem_by_address(uint8_t* args, uint16_t arg_len);
        void process_tester_present(uint8_t* args, uint16_t arg_len);
        void process_control_dtc_settings(uint8_t* args, uint16_t arg_len);
        void process_response_on_event(uint8_t* args, uint16_t arg_len);

        void make_diag_neg_msg(uint8_t sid, uint8_t nrc);
        void make_diag_pos_msg(uint8_t sid, uint8_t* resp, uint16_t len);

        Gearbox* gearbox_ptr;
        AbstractCan* can_layer;
};

#endif //_KWP_H__