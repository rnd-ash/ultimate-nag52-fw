#include "shift_report.h"
#include "string.h"
#include "esp_flash.h"
#include "esp_partition.h"
#include "esp_log.h"

#define NVS_NAME_SS "SHIFT_STORE"

uint16_t calc_shift_report_group_crc(ShiftReportNvsGroup* rpt) {
    size_t byte_count = sizeof(ShiftReportNvsGroup) - 2; // CRC MUST be at the end
    uint16_t total = 0;
    uint8_t* ptr = (uint8_t*)(rpt);
    for (int i = 0; i < byte_count; i++) {
        total += ptr[i] + i;
    }
    return total;
}

bool load_shift_report_from_partition(ShiftReportNvsGroup* dest, uint32_t addr) {
    esp_err_t e = esp_flash_read(esp_flash_default_chip, dest, addr, sizeof(ShiftReportNvsGroup));
    if (e != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "SHIFT_REPORTER", "Error reading SPIFFS for shift data: %s", esp_err_to_name(e));
    }
    uint16_t raw_crc = calc_shift_report_group_crc(dest);
    if (dest->crc != raw_crc) {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT_REPORT", "Load from SPIFFS was invalid, resetting. CRC was %08X, wanted CRC: %08X", dest->crc, raw_crc);
        memset(dest, 0x00, sizeof(ShiftReportNvsGroup)); // Blank it if invalid!
        return false;
    }
    return true;
}

bool save_shift_report_to_partition(ShiftReportNvsGroup* src, uint32_t addr) {
    src->crc = calc_shift_report_group_crc(src);
    esp_err_t e = esp_flash_write(esp_flash_default_chip, src, addr, sizeof(ShiftReportNvsGroup));
    if (e != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "SHIFT_REPORTER", "Error writing shift data to SPIFFS: %s", esp_err_to_name(e));
    }
    return e == ESP_OK;
}


ShiftReporter::ShiftReporter() {
    this->report_group = (ShiftReportNvsGroup*)heap_caps_malloc(sizeof(ShiftReportNvsGroup), MALLOC_CAP_SPIRAM);
    if (this->report_group == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SHIFT_REPORT", "Cannot allocate memory!");
        return;
    }

    const esp_partition_t *p = esp_partition_find_first(esp_partition_type_t::ESP_PARTITION_TYPE_DATA, esp_partition_subtype_t::ESP_PARTITION_SUBTYPE_DATA_NVS, "tcm_shift_store");
    if (p != nullptr) {
        this->spiffs_addr = p->address;
        if (load_shift_report_from_partition(this->report_group, this->spiffs_addr)) {
            ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFT_REPORT", "Load from SPIFFS OK!");
        } else {
            ESP_LOG_LEVEL(ESP_LOG_WARN, "SHIFT_REPORT", "Load from SPIFFS was invalid. Resetting data");
        }
    } else {
        this->spiffs_addr = 0;
        ESP_LOG_LEVEL(ESP_LOG_WARN, "SHIFT_REPORT", "Partition not found! Shift reports won't be saved");
    }
}

ShiftReporter::~ShiftReporter() {
}

void ShiftReporter::add_report(ShiftReport src) {
    if (this->report_group == nullptr) {
        return;
    }
    // Which index to we write to?
    if (this->report_group->index >= MAX_REPORTS) {
        this->report_group->index = 0;
    }
    uint8_t idx = this->report_group->index;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Writing to report %d as writable", idx);
    this->report_group->reports[idx] = src;
    this->report_group->index++;
    this->has_change = true;
}


void ShiftReporter::save() {
    if (!has_change || this->report_group == nullptr || this->spiffs_addr == 0) {
        return;
    }
    save_shift_report_to_partition(this->report_group, this->spiffs_addr);
}