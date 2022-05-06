#include "flasher.h"
#include "esp_timer.h"
#include "speaker.h"
#include "esp_partition.h"
#include "tcm_maths.h"

Flasher::Flasher(AbstractCan *can_ref, Gearbox* gearbox) {
    this->can_ref = can_ref;
    this->gearbox_ref = gearbox;
}

Flasher::~Flasher() {
    this->gearbox_ref->diag_regain_control(); // Re-enable engine starting
}

DiagMessage Flasher::on_request_download(uint8_t* args, uint16_t arg_len) {
    uint32_t now = esp_timer_get_time()/1000;
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
    ESP_LOGI("FLASHER", "DEST %08X, Format is %02X and size is %08X", dest_mem_address, fmt, dest_mem_size);
    // Valid memory regions
    if (dest_mem_address == MEM_REGION_OTA) {
        // Init OTA system
        this->update_partition = esp_ota_get_next_update_partition(NULL);
        this->update_type = UPDATE_TYPE_OTA;
        if (this->update_partition == nullptr) {
            ESP_LOGE("FLASHER", "Target update partition was NULL!");
            return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_GENERAL_REJECT);
        }
        this->update_handle = 0;
        esp_err_t err = esp_ota_begin(this->update_partition, dest_mem_size, &update_handle);
        if (err != ESP_OK) {
            // HUH!?
            ESP_LOGE("FLASHER", "esp_ota_begin failed! %s", esp_err_to_name(err));
            esp_ota_abort(this->update_handle);
            return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_GENERAL_REJECT);
        }
    } else { // Invalid memory region
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
    // Shifter must be Offline (SNV) or P or N
    if (!is_shifter_passive(this->can_ref)) {
        ESP_LOGE("FLASHER", "Rejecting download request. Shifter not in valid position");
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
    }
    if (!is_engine_off(this->can_ref)) {
        ESP_LOGE("FLASHER", "Rejecting download request. Engine is on");
        return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
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
    spkr.send_note(1000, 100, 100);
    this->written_data = 0;
    this->data_dir = DATA_DIR_DOWNLOAD;
    return this->make_diag_pos_msg(SID_REQ_DOWNLOAD, resp, 2);
}

DiagMessage Flasher::on_request_upload(uint8_t* args, uint16_t arg_len) {
    uint32_t now = esp_timer_get_time()/1000;
    // Format [MA, MA, MA, FF, MS, MS, MS]
    // Dest memory address
    // data format
    // Uncompressed memory size
    // For now we only support 0x00 format (Uncompressed and unencrypted)
    ESP_LOGI("FLASHER", "Upload requested. Arg size %d", arg_len);
    if (arg_len != 7) { // Request was the wrong size
        return this->make_diag_neg_msg(SID_REQ_UPLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
    uint32_t src_mem_address = args[0] << 16 | args[1] << 8 | args[2];
    uint8_t fmt = args[3];
    uint32_t src_mem_size = args[4] << 16 | args[5] << 8 | args[6];
    ESP_LOGI("FLASHER", "DEST %08X, Format is %02X and size is %08X", src_mem_address, fmt, src_mem_size);
    // Valid memory regions
    if (src_mem_address == MEM_REGION_COREDUMP) {
        this->update_type = UPDATE_TYPE_COREDUMP;
        ESP_LOGI("FLASHER", "Upload coredump requested");
    } else { // Invalid memory region
        return this->make_diag_neg_msg(SID_REQ_UPLOAD, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
    }
    // Shifter must be Offline (SNV) or P or N
    ShifterPosition pos = this->can_ref->get_shifter_position_ewm(now, 250);
    if (
        pos == ShifterPosition::D || pos == ShifterPosition::MINUS || pos == ShifterPosition::PLUS || pos == ShifterPosition::R || // Stationary positions
        pos == ShifterPosition::N_D || pos == ShifterPosition::P_R || pos == ShifterPosition::R_N // Intermediate positions
        ) {
            ESP_LOGE("FLASHER", "Rejecting download request. Shifter not in valid position");
            return this->make_diag_neg_msg(SID_REQ_UPLOAD, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
    }
    // Engine MUST be off (Ignition state)
    int rpm = egs_can_hal->get_engine_rpm(now, 250);
    if (rpm != 0 && rpm != UINT16_MAX) { // 0 = 0RPM, MAX = SNV (Engine ECU is offline)
        ESP_LOGE("FLASHER", "Rejecting download request. Engine RPM is %d", rpm);
        return this->make_diag_neg_msg(SID_REQ_UPLOAD, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
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
    spkr.send_note(1000, 100, 100);
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
                // Write to OTA partiton
                if (esp_ota_write(this->update_handle, (const void*)&args[1], arg_len-1) == ESP_OK) {
                    this->written_data += arg_len-1;
                    return this->make_diag_pos_msg(SID_TRANSFER_DATA, nullptr, 0);
                } else {
                    ESP_LOGE("FLASHER", "esp_ota_write failed!");
                    esp_ota_abort(this->update_handle);
                    return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_GENERAL_REJECT);
                }
            } else {
                ESP_LOGI("FLASHER", "OTA invalid type!");
                return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT);
            }
        } else if (args[0] == this->block_counter) {
            // Repeated request, KWP spec says to do nothing and just return OK!
            ESP_LOGI("FLASHER", "Skip write!");
            return this->make_diag_pos_msg(SID_TRANSFER_DATA, nullptr, 0);
        } else {
            // Huh
            ESP_LOGI("FLASHER", "Wtf");
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
            return this->make_diag_neg_msg(SID_TRANSFER_DATA, NRC_GENERAL_REJECT);
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
            return this->make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR); // Can't start verirication without it!
        } else {
            /// Now verify and switch boot partitions!
            uint8_t res[2] = {0xE1, 0x00};
            esp_err_t e = esp_ota_end(this->update_handle);
            if (e != ESP_OK) {
                res[1] = FLASH_CHECK_STATUS_INVALID;
                ESP_LOGE("FLASHER", "Flash check failed! %s", esp_err_to_name(e));
                return this->make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2); // TODO
            }
            e = esp_ota_set_boot_partition(this->update_partition);
            if (e != ESP_OK) {
                res[1] = FLASH_CHECK_STATUS_INVALID;
                ESP_LOGE("FLASHER", "Set boot partition failed! %s", esp_err_to_name(e));
                return this->make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2); // TODO
            }
            // OK!
            res[1] = FLASH_CHECK_STATUS_OK;
            ESP_LOGI("FLASHER", "All done!");
            return this->make_diag_pos_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, res, 2); // TODO
        }
    } else {
        return this->make_diag_neg_msg(SID_START_ROUTINE_BY_LOCAL_IDENT, NRC_GENERAL_REJECT); // TODO
    }
}