

#ifndef _KWP_H__
#define _KWP_H__

#include "endpoints/endpoint.h"
#include <stdint.h>
#include "kwp2000_defines.h"
#include "gearbox.h"
#include "canbus/can_hal.h"
#include "gearbox_config.h"
#include "perf_mon.h"
#include "esp32/spiram.h"
#include "flasher.h"

#define PROCESSOR_TYPE
#define COMM_MATRIX_VERSION 0x0101 // 01.01
#define CAN_DRIVER_VERSION 0x0101 // 01.01
#define NM_VERSION 0x0101 // 01.01
#define KWP_MOD_VERSION 0x0001 // 00.01
#define TP_LAYER_VERSION 0x0001 // 00.01
#define DBKOM_VERSION 0x0001 // 00.01
#define FLEXER_VERSION 0x9999 // 99.99 (unsupported)

#define ROUTINE_ADAPTATION_RESET 0xDD
#define ROUTINE_SOLENOID_TEST 0xDE
#define ROUTINE_SPEAKER_TEST 0xDF

class Kwp2000_server {
    public:
        Kwp2000_server(EgsBaseCan* can_layer, Gearbox* gearbox);
        ~Kwp2000_server();

        static void start_kwp_server(void *_this) {
            static_cast<Kwp2000_server*>(_this)->server_loop();
        }

        kwp_result_t convert_err_result(kwp_result_t in);

        void make_diag_neg_msg(uint8_t sid, kwp_result_t nrc);
        void make_diag_pos_msg(uint8_t sid, const uint8_t* resp, uint16_t len);
        void make_diag_pos_msg(uint8_t sid, uint8_t pid, const uint8_t* resp, uint16_t len);
        Gearbox* gearbox_ptr;
        EgsBaseCan* can_layer;
    private:
        [[noreturn]]
        void server_loop();
        uint8_t session_mode;
        uint64_t next_tp_time;
        DiagMessage tx_msg;
        DiagMessage rx_msg;
        UsbEndpoint* usb_diag_endpoint;
        CanEndpoint* can_endpoint;
        bool routine_running = false;
        TaskHandle_t routine_task;
        uint8_t routine_id = 0x00;
        uint8_t routine_result[255];
        uint8_t* running_routine_args;
        uint8_t routine_results_len = 0;
        bool send_resp;
        bool reboot_pending;

        int allocate_routine_args(uint8_t* src, uint8_t arg_len);
        void process_start_diag_session(const uint8_t* args, uint16_t arg_len);
        void process_ecu_reset(const uint8_t* args, uint16_t arg_len);
        void process_clear_diag_info(uint8_t* args, uint16_t arg_len);
        void process_read_status_of_dtcs(uint8_t* args, uint16_t arg_len);
        void process_read_ecu_ident(const uint8_t* args, uint16_t arg_len);
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
        void process_request_routine_results_by_local_ident(const uint8_t* args, uint16_t arg_len);
        void process_request_download(uint8_t* args, uint16_t arg_len);
        void process_request_upload(uint8_t* args, uint16_t arg_len);
        void process_transfer_data(uint8_t* args, uint16_t arg_len);
        void process_transfer_exit(uint8_t* args, uint16_t arg_len);
        void process_write_data_by_local_ident(uint8_t* args, uint16_t arg_len);
        void process_write_mem_by_address(uint8_t* args, uint16_t arg_len);
        void process_tester_present(const uint8_t* args, uint16_t arg_len);
        void process_control_dtc_settings(uint8_t* args, uint16_t arg_len);
        void process_response_on_event(uint8_t* args, uint16_t arg_len);
        void process_shift_mgr_op(uint8_t* args, uint16_t arg_len);
        
        static void launch_solenoid_test(void *_this) {
            static_cast<Kwp2000_server*>(_this)->run_solenoid_test();
        }

        static void launch_speaker_test(void *_this) {
            static_cast<Kwp2000_server*>(_this)->run_solenoid_test();
        }

        void run_solenoid_test();
        xTaskHandle* running_routine;
        Flasher* flash_handler = nullptr;
        uint8_t supplier_id;
        uint16_t diag_var_code;
};

#endif //_KWP_H__