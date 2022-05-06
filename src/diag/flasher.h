#ifndef __FLASHER_H_
#define __FLASHER_H_

#include "kwp2000_defines.h"
#include "canbus/can_hal.h"
#include "gearbox.h"
#include "kwp_utils.h"
#include "esp_ota_ops.h"

#define CHUNK_SIZE 2040 // 254 byte chunks from KWP get sent to OTA (1 extra byte for block counter)

#define FLASH_CHECK_STATUS_OK 0x00
#define FLASH_CHECK_STATUS_INVALID 0x01

#define MEM_REGION_OTA      0x000000
#define MEM_REGION_COREDUMP 0x310000

#define UPDATE_TYPE_OTA 0x00 // Read and write
#define UPDATE_TYPE_COREDUMP 0x01 // Read only

#define DATA_DIR_UPLOAD 0x02
#define DATA_DIR_DOWNLOAD 0x01

class Flasher {
    public:
        Flasher(AbstractCan *can_ref, Gearbox* gearbox);
        ~Flasher();
        DiagMessage on_request_download(uint8_t* args, uint16_t arg_len);
        DiagMessage on_request_upload(uint8_t* args, uint16_t arg_len);
        DiagMessage on_transfer_data(uint8_t* args, uint16_t arg_len);
        DiagMessage on_transfer_exit(uint8_t* args, uint16_t arg_len);
        DiagMessage on_request_verification(uint8_t* args, uint16_t arg_len);
    private:
        Gearbox* gearbox_ref;
        AbstractCan* can_ref;
        uint8_t block_counter = 0;
        uint8_t update_type = 0;
        uint32_t data_read = 0;
        esp_ota_handle_t update_handle = 0;
        uint32_t written_data = 0;
        const esp_partition_t *update_partition;
        int data_dir = 0;

        // For reading
        size_t read_base_addr;
        size_t read_bytes;
        size_t read_bytes_total;

        DiagMessage make_diag_neg_msg(uint8_t sid, uint8_t nrc){
            DiagMessage msg;
            global_make_diag_neg_msg(&msg, sid, nrc);
            return msg;
        };

        DiagMessage make_diag_pos_msg(uint8_t sid, const uint8_t* resp, uint16_t len){
             DiagMessage msg;
             global_make_diag_pos_msg(&msg, sid, resp, len);
             return msg;
        };
};

#endif