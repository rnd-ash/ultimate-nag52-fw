#ifndef __FLASHER_H_
#define __FLASHER_H_

#include "kwp2000_defines.h"
#include "canbus/can_hal.h"
#include "gearbox.h"
#include "kwp_utils.h"
#include "esp_ota_ops.h"

#define CHUNK_SIZE_USB 4093 // 1024 byte chunks from KWP get sent to OTA (1 extra byte for block counter)
#define CHUNK_SIZE_CAN 254

static_assert(CHUNK_SIZE_USB+2 <= DIAG_CAN_MAX_SIZE);
static_assert(CHUNK_SIZE_CAN+2 <= DIAG_CAN_MAX_SIZE);

#define FLASH_CHECK_STATUS_OK 0x00
#define FLASH_CHECK_STATUS_INVALID 0x01

#define MEM_REGION_OTA      0x000000
#define MEM_REGION_COREDUMP 0x310000

#define UPDATE_TYPE_OTA 0x00 // Read and write

#define DATA_DIR_UPLOAD 0x02
#define DATA_DIR_DOWNLOAD 0x01

#define FMT_OTA 0xF0

class Flasher {
    public:
        Flasher(EgsBaseCan *can_ref, Gearbox* gearbox);
        ~Flasher();
        void on_request_download(const uint8_t* args, uint16_t arg_len, DiagMessage* dest, bool using_can);
        void on_request_upload(const uint8_t* args, uint16_t arg_len, DiagMessage* dest, bool using_can);
        void on_transfer_data(uint8_t* args, uint16_t arg_len, DiagMessage* dest, bool using_can);
        void on_transfer_exit(uint8_t* args, uint16_t arg_len, DiagMessage* dest);
        void on_request_verification(uint8_t* args, uint16_t arg_len, DiagMessage* dest);
    private:
        Gearbox* gearbox_ref;
        EgsBaseCan* can_ref;
        uint8_t block_counter = 0;
        uint32_t data_read = 0;
        //esp_ota_handle_t update_handle = 0;
        uint32_t written_data = 0;
        uint32_t start_addr = 0;
        uint32_t to_write = 0;
        //const esp_partition_t *update_partition;
        int data_dir = 0;

        // For reading
        size_t read_base_addr;
        size_t read_bytes;
        size_t read_bytes_total;
        bool is_ota = false;
};

#endif