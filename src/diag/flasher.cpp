#include "flasher.h"
#include "esp_timer.h"
#include "speaker.h"
#include "esp_partition.h"
#include "tcu_maths.h"
#include "esp_ota_ops.h"
#include "esp_flash.h"
#include "esp_image_format.h"
Flasher::Flasher(EgsBaseCan *can_ref, Gearbox* gearbox) {
    this->can_ref = can_ref;
    this->gearbox_ref = gearbox;
    read_base_addr = 0u;
    read_bytes = 0u;
    read_bytes_total = 0u;
}

Flasher::~Flasher() {
    this->gearbox_ref->diag_regain_control(); // Re-enable engine starting
}

/**
 * "This is a function that handles a Request Download diagnostic message in a vehicle's 
 *  onboard diagnostic system. The function checks the current state of the vehicle's shifter 
 *  and engine, and then parses the request data to determine the destination memory address, 
 *  data format, and uncompressed memory size. If the request is valid, it prepares the vehicle 
 *  for data transfer by disabling the gearbox controller and setting the appropriate drive 
 *  profile and display gear. It also sets up a block counter to track the progress of the data 
 *  transfer. The function then returns a positive response message containing the maximum 
 *  number of data bytes that can be transferred in a single block." - ChatGPT
*/
void Flasher::on_request_download(const uint8_t* args, uint16_t arg_len, DiagMessage* dest) {
    // Shifter must be Offline (SNV) or P or N
    if (!is_shifter_passive(this->can_ref)) {
        global_make_diag_neg_msg(dest, SID_REQ_DOWNLOAD, NRC_UN52_SHIFTER_ACTIVE);
        return;
    }
    if (!is_engine_off(this->can_ref)) {
        global_make_diag_neg_msg(dest, SID_REQ_DOWNLOAD, NRC_UN52_ENGINE_ON);
        return;
    }
    // Format [MA, MA, MA, FF, MS, MS, MS]
    // Dest memory address
    // data format
    // Uncompressed memory size
    // For now we only support 0x00 format (Uncompressed and unencrypted)
    if (arg_len != 7) { // Request was the wrong size
        global_make_diag_neg_msg(dest, SID_REQ_DOWNLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        return;
    }
    uint32_t dest_mem_address = args[0] << 16 | args[1] << 8 | args[2];
    uint8_t fmt = args[3];
    uint32_t dest_mem_size = args[4] << 16 | args[5] << 8 | args[6];
    // Valid memory regions
    uint32_t flash_size;
    if (esp_flash_get_size(esp_flash_default_chip, &flash_size) != ESP_OK) {
        global_make_diag_neg_msg(dest, SID_REQ_DOWNLOAD, NRC_GENERAL_REJECT);
        return;
    }
    if (dest_mem_address+dest_mem_size > flash_size) {
        global_make_diag_neg_msg(dest, SID_REQ_DOWNLOAD, NRC_GENERAL_REJECT);
        return;
    }
    // Must be 4096 byte sector aligned
    int erase_len = (dest_mem_size + 4096 - 1) & -4096;
    if (esp_flash_erase_region(esp_flash_default_chip, dest_mem_address, erase_len) != ESP_OK) {
        global_make_diag_neg_msg(dest, SID_REQ_DOWNLOAD, NRC_GENERAL_REJECT);
        return;
    }

    const esp_partition_t* part_info_for_ota = esp_ota_get_next_update_partition(nullptr);
    if (part_info_for_ota != nullptr && part_info_for_ota->address == dest_mem_address) {
        // Erase coredump for an OTA. This stops old coredumps from hanging around
        esp_core_dump_image_erase();
    }

    this->start_addr = dest_mem_address;
    this->to_write = dest_mem_size;

    // Ok, conditions are correct, now we need to prepare
    this->gearbox_ref->diag_inhibit_control(); // Disable gearbox controller
    vTaskDelay(50);
    this->can_ref->set_safe_start(false);
    this->can_ref->set_drive_profile(GearboxProfile::Underscore);
    this->can_ref->set_display_gear(GearboxDisplayGear::SNA, false);
    this->block_counter = 0;
    uint8_t resp[2] =  { 0x00, 0x00 };
    resp[0] = CHUNK_SIZE >> 8 & 0xFF;
    resp[1] = CHUNK_SIZE & 0xFF;
    this->written_data = 0;
    this->is_ota = (fmt & FMT_OTA) != 0;
    this->data_dir = DATA_DIR_DOWNLOAD;
    global_make_diag_pos_msg(dest, SID_REQ_DOWNLOAD, resp, 2);
}

void Flasher::on_request_upload(const uint8_t* args, uint16_t arg_len, DiagMessage* dest) {
    // Shifter must be Offline (SNV) or P or N
    if (!is_shifter_passive(this->can_ref)) {
        return global_make_diag_neg_msg(dest, SID_REQ_DOWNLOAD, NRC_UN52_SHIFTER_ACTIVE);
    }
    if (!is_engine_off(this->can_ref)) {
        return global_make_diag_neg_msg(dest, SID_REQ_DOWNLOAD, NRC_UN52_ENGINE_ON);
    }
    // Format [MA, MA, MA, FF, MS, MS, MS]
    // Dest memory address
    // data format
    // Uncompressed memory size
    // For now we only support 0x00 format (Uncompressed and unencrypted)
    if (arg_len != 7) { // Request was the wrong size
        return global_make_diag_neg_msg(dest, SID_REQ_UPLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
    uint32_t src_mem_address = args[0] << 16 | args[1] << 8 | args[2];
    uint32_t src_mem_size = args[4] << 16 | args[5] << 8 | args[6]; 
    // Valid memory regions
    uint32_t flash_size;
    if (esp_flash_get_size(esp_flash_default_chip, &flash_size) != ESP_OK) {
        return global_make_diag_neg_msg(dest, SID_REQ_UPLOAD, NRC_GENERAL_REJECT);
    }
    if (src_mem_address + src_mem_size > flash_size) { // Invalid memory
        return global_make_diag_neg_msg(dest, SID_REQ_UPLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
    // Ok, conditions are correct, now we need to prepare
    this->gearbox_ref->diag_inhibit_control(); // Disable gearbox controller
    vTaskDelay(50);
    this->can_ref->set_safe_start(false);
    this->can_ref->set_drive_profile(GearboxProfile::Underscore);
    this->can_ref->set_display_gear(GearboxDisplayGear::SNA, false);
    this->block_counter = 0;
    uint8_t resp[2] =  { 0x00, 0x00 };
    resp[0] = CHUNK_SIZE >> 8 & 0xFF;
    resp[1] = CHUNK_SIZE & 0xFF;
    this->data_dir = DATA_DIR_UPLOAD;
    this->read_base_addr = src_mem_address;
    this->read_bytes = 0;
    this->read_bytes_total = src_mem_size;
    return global_make_diag_pos_msg(dest, SID_REQ_UPLOAD, resp, 2);
}


void Flasher::on_transfer_data(uint8_t* args, uint16_t arg_len, DiagMessage* dest) {
    if (this->data_dir == DATA_DIR_DOWNLOAD) {
        // We use block sequence counter
        if (arg_len < 2) {
            global_make_diag_neg_msg(dest, SID_TRANSFER_DATA, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            return;
        }
        if (args[0] == this->block_counter+1 || (args[0] == 0x00 && this->block_counter == 0xFF)) {
            // Next block
            this->block_counter++;
            if (esp_flash_write(esp_flash_default_chip, (const void*)&args[1], this->start_addr + this->written_data, arg_len-1) != ESP_OK) {
                global_make_diag_neg_msg(dest, SID_TRANSFER_DATA, NRC_UN52_OTA_WRITE_FAIL);
                return;
            } else {
                this->written_data += arg_len-1;
                global_make_diag_pos_msg(dest, SID_TRANSFER_DATA, nullptr, 0);
                return;
            }
        } else if (args[0] == this->block_counter) {
            // Repeated request, KWP spec says to do nothing and just return OK!
            global_make_diag_pos_msg(dest, SID_TRANSFER_DATA, nullptr, 0);
            return;
        } else {
            // Huh
            global_make_diag_neg_msg(dest, SID_TRANSFER_DATA, NRC_GENERAL_REJECT);
            return;
        }
    } else if (this->data_dir == DATA_DIR_UPLOAD) {
        if (this->read_bytes >= this->read_bytes_total) {
            global_make_diag_neg_msg(dest, SID_TRANSFER_DATA, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
            return;
        }
        // Construct diag message manually!
        dest->data[0] = SID_TRANSFER_DATA+0x40;
        dest->data[1] = args[0];
        uint32_t max_bytes = MIN(CHUNK_SIZE, (uint32_t)(read_bytes_total-read_bytes));
        // Make diag message manually
        esp_flash_read(esp_flash_default_chip, &dest->data[2], this->read_base_addr+this->read_bytes, max_bytes);
        dest->id = 0x07E9;
        dest->data_size = 2+max_bytes;
        this->read_bytes += max_bytes;
        return;
    } else { // Transfer mode not set!
        return global_make_diag_neg_msg(dest, SID_TRANSFER_DATA, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
    }
}

void Flasher::on_transfer_exit(uint8_t* args, uint16_t arg_len, DiagMessage* dest) {
    this->data_dir = 0; // Invalidate it
    // Return control back to TCM
    this->gearbox_ref->diag_regain_control();
    global_make_diag_pos_msg(dest, SID_TRANSFER_EXIT, nullptr, 0x00);
}

void Flasher::on_request_verification(uint8_t* args, uint16_t arg_len, DiagMessage* dest) {
    uint8_t res[2] = {0xE1, 0x00};
    if (this->is_ota) {
        // Only for OTA update
        esp_image_metadata_t data;
        const esp_partition_t* part = esp_ota_get_next_update_partition(NULL);
        const esp_partition_pos_t part_pos = {
            .offset = part->address,
            .size = part->size,
        };
        esp_err_t e = esp_image_verify(ESP_IMAGE_VERIFY, &part_pos, &data);
        // uint8_t res[2] = {0xE1, 0x00};
        if (e != ESP_OK) {
            res[1] = FLASH_CHECK_STATUS_INVALID;
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "FLASHER", "Flash check failed! %s", esp_err_to_name(e));
            return global_make_diag_pos_msg(dest, SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2);
        }
        e = esp_ota_set_boot_partition(part);
        if (e != ESP_OK) {
            res[1] = FLASH_CHECK_STATUS_INVALID;
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "FLASHER", "Set boot partition failed! %s", esp_err_to_name(e));
            return global_make_diag_pos_msg(dest, SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2);
        }
        res[1] = FLASH_CHECK_STATUS_OK;
        ESP_LOG_LEVEL(ESP_LOG_INFO, "FLASHER", "All done!");
    } else {
        res[1] = FLASH_CHECK_STATUS_OK;
    }
    return global_make_diag_pos_msg(dest, SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2);
}