#include "flasher.h"
#include "esp_timer.h"
#include "speaker.h"

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
        const esp_partition_t *target_update_partition = esp_ota_get_next_update_partition(NULL);
        this->update_type = UPDATE_TYPE_OTA;
        if (target_update_partition == nullptr) {
            ESP_LOGE("FLASHER", "Target update partition was NULL!");
            return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_GENERAL_REJECT);
        }
        this->update_handle = 0;
        this->update_partition = target_update_partition;
        esp_err_t err = esp_ota_begin(target_update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
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
    ShifterPosition pos = this->can_ref->get_shifter_position_ewm(now, 250);
    if (
        pos == ShifterPosition::D || pos == ShifterPosition::MINUS || pos == ShifterPosition::PLUS || pos == ShifterPosition::R || // Stationary positions
        pos == ShifterPosition::N_D || pos == ShifterPosition::P_R || pos == ShifterPosition::R_N // Intermediate positions
        ) {
            ESP_LOGE("FLASHER", "Rejecting download request. Shifter not in valid position");
            return this->make_diag_neg_msg(SID_REQ_DOWNLOAD, NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR);
    }
    // Engine MUST be off (Ignition state)
    int rpm = egs_can_hal->get_engine_rpm(now, 250);
    if (rpm != 0 && rpm != UINT16_MAX) { // 0 = 0RPM, MAX = SNV (Engine ECU is offline)
        ESP_LOGE("FLASHER", "Rejecting download request. Engine RPM is %d", rpm);
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
    return this->make_diag_pos_msg(SID_REQ_DOWNLOAD, resp, 2);
}

DiagMessage Flasher::on_request_upload(uint8_t* args, uint16_t arg_len) {
    uint32_t now = esp_timer_get_time()/1000;
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
    return this->make_diag_pos_msg(SID_REQ_UPLOAD, resp, 2);
}


DiagMessage Flasher::on_transfer_data(uint8_t* args, uint16_t arg_len) {
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
                ESP_LOGE("FLASHER", "Written data %d", this->written_data);
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
}

DiagMessage Flasher::on_transfer_exit(uint8_t* args, uint16_t arg_len) {
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