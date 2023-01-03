#include "flasher.h"
#include "esp_timer.h"
#include "speaker.h"
#include "esp_partition.h"
#include "tcu_maths.h"

Flasher::Flasher(EgsBaseCan *can_ref, Gearbox* gearbox) {
    this->can_ref = can_ref;
    this->gearbox_ref = gearbox;
    update_partition = nullptr;
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
DiagMessage Flasher::on_request_download(const uint8_t* args, uint16_t arg_len) {
    // Shifter must be Offline (SNV) or P or N
    if (!is_shifter_passive(this->can_ref)) {
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_UN52_SHIFTER_ACTIVE);
    }
    if (!is_engine_off(this->can_ref)) {
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_UN52_ENGINE_ON);
    }
    // Format [MA, MA, MA, FF, MS, MS, MS]
    // Dest memory address
    // data format
    // Uncompressed memory size
    // For now we only support 0x00 format (Uncompressed and unencrypted)
    if (arg_len != 7) { // Request was the wrong size
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
    uint32_t dest_mem_address = args[0] << 16 | args[1] << 8 | args[2];
    uint8_t fmt = args[3];
    uint32_t dest_mem_size = args[4] << 16 | args[5] << 8 | args[6];
    // Valid memory regions
    if (dest_mem_address == MEM_REGION_OTA) {
        // Init OTA system
        this->update_partition = esp_ota_get_next_update_partition(NULL);
        this->update_type = UPDATE_TYPE_OTA;
        if (this->update_partition == nullptr) {
            return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_UN52_NULL_PARTITION);
        }
        this->update_handle = 0;
        esp_err_t err = esp_ota_begin(this->update_partition, dest_mem_size, &update_handle);
        if (err != ESP_OK) {
            esp_ota_abort(this->update_handle);
            return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_UN52_OTA_BEGIN_FAIL);
        }
    } else { // Invalid memory region
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
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
    this->written_data = 0;
    this->data_dir = DATA_DIR_DOWNLOAD;
    return this->make_diag_pos_msg(SID_REQ_DOWNLOAD, resp, 2);
}

DiagMessage Flasher::on_request_upload(const uint8_t* args, uint16_t arg_len) {
    // Shifter must be Offline (SNV) or P or N
    if (!is_shifter_passive(this->can_ref)) {
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_UN52_SHIFTER_ACTIVE);
    }
    if (!is_engine_off(this->can_ref)) {
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_UN52_ENGINE_ON);
    }
    // Format [MA, MA, MA, FF, MS, MS, MS]
    // Dest memory address
    // data format
    // Uncompressed memory size
    // For now we only support 0x00 format (Uncompressed and unencrypted)
    if (arg_len != 7) { // Request was the wrong size
        return this->make_diag_neg_msg(SID_REQ_UPLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
    uint32_t src_mem_address = args[0] << 16 | args[1] << 8 | args[2];
    uint8_t fmt = args[3];
    uint32_t src_mem_size = args[4] << 16 | args[5] << 8 | args[6];
    // Valid memory regions
    if (src_mem_address == MEM_REGION_COREDUMP) {
        this->update_type = UPDATE_TYPE_COREDUMP;
        ESP_LOG_LEVEL(ESP_LOG_INFO, "FLASHER", "Upload coredump requested");
    } else { // Invalid memory region
        return this->make_diag_neg_msg(SID_REQ_UPLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
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
    spkr->send_note(1000, 100, 100);
    this->data_dir = DATA_DIR_UPLOAD;
    this->read_base_addr = src_mem_address;
    this->read_bytes = 0;
    this->read_bytes_total = src_mem_size;
    return this->make_diag_pos_msg(SID_REQ_UPLOAD, resp, 2);
}


DiagMessage Flasher::on_transfer_data(uint8_t* args, uint16_t arg_len) {
    if (this->data_dir == DATA_DIR_DOWNLOAD) {
        // We use block sequence counter
        if (arg_len < 2) {
            return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
        }
        if (args[0] == this->block_counter+1 || (args[0] == 0x00 && this->block_counter == 0xFF)) {
            // Next block
            this->block_counter++;
            if (this->update_type == UPDATE_TYPE_OTA) {
                // Write to OTA partition
                if (esp_ota_write(this->update_handle, (const void*)&args[1], arg_len-1) == ESP_OK) {
                    this->written_data += arg_len-1;
                    return this->make_diag_pos_msg(SID_TRANSFER_DATA, nullptr, 0);
                } else {
                    esp_ota_abort(this->update_handle);
                    return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_UN52_OTA_WRITE_FAIL);
                }
            } else {
                return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_UN52_OTA_INVALID_TY);
            }
        } else if (args[0] == this->block_counter) {
            // Repeated request, KWP spec says to do nothing and just return OK!
            return this->make_diag_pos_msg(SID_TRANSFER_DATA, nullptr, 0);
        } else {
            // Huh
            return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_GENERAL_REJECT);
        }
    } else if (this->data_dir == DATA_DIR_UPLOAD) {
        if (this->update_type == UPDATE_TYPE_COREDUMP) {
            if (this->read_bytes >= this->read_bytes_total) {
                return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
            }
            DiagMessage msg;
            msg.data[0] = SID_TRANSFER_DATA+0x40;
            msg.data[1] = args[0];
            uint32_t max_bytes = MIN(CHUNK_SIZE, (uint32_t)(read_bytes_total-read_bytes));
            // Make diag message manually
            esp_flash_read(esp_flash_default_chip, &msg.data[2], this->read_base_addr+this->read_bytes ,max_bytes);
            msg.id = 0x07E9;
            msg.data_size = 2+max_bytes;
            this->read_bytes += max_bytes;
            return msg;
        } else {
            return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_UN52_OTA_INVALID_TY);
        }
    } else { // Transfer mode not set!
        return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
    }
}

DiagMessage Flasher::on_transfer_exit(uint8_t* args, uint16_t arg_len) {
    this->data_dir = 0; // Invalidate it

    // Return control back to TCM
    this->gearbox_ref->diag_regain_control();
    return this->make_diag_pos_msg(SID_TRANSFER_EXIT, nullptr, 0x00);
}

DiagMessage Flasher::on_request_verification(uint8_t* args, uint16_t arg_len) {
    if (this->update_type == UPDATE_TYPE_OTA) {
        if (this->update_handle == 0) {
            return this->make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR); // Can't start verification without it!
        } else {
            /// Now verify and switch boot partitions!
            uint8_t res[2] = {0xE1, 0x00};
            esp_err_t e = esp_ota_end(this->update_handle);
            if (e != ESP_OK) {
                res[1] = FLASH_CHECK_STATUS_INVALID;
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "FLASHER", "Flash check failed! %s", esp_err_to_name(e));
                return this->make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2); // TODO
            }
            e = esp_ota_set_boot_partition(this->update_partition);
            if (e != ESP_OK) {
                res[1] = FLASH_CHECK_STATUS_INVALID;
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "FLASHER", "Set boot partition failed! %s", esp_err_to_name(e));
                return this->make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2); // TODO
            }
            // OK!
            res[1] = FLASH_CHECK_STATUS_OK;
            ESP_LOG_LEVEL(ESP_LOG_INFO, "FLASHER", "All done!");
            return this->make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2); // TODO
        }
    } else {
        return this->make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_GENERAL_REJECT); // TODO
    }
}