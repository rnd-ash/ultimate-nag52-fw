#include "shift_report.h"
#include "string.h"

#define NVS_NAME_SS "SHIFT_STORE"

ShiftReporter::ShiftReporter() {
    bool failure = true;
    if (nvs_open("tcm_shift_store", NVS_READWRITE, &this->nvs_handle) != ESP_OK) {
        ESP_LOGE("SHIFT_REPORTER","Cannot open TCM Shift store partition, no data will be saved");
        return;
    }

    // Ok so now we have our default grp structure,
    // Try to allocate memory
    size_t s = sizeof(ShiftReportNvsGroup);
    esp_err_t e = nvs_get_blob(this->nvs_handle, NVS_NAME_SS, &this->report_group, &s);

    if (e == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE("SHIFT_REPORTER", "History not found in NVS, Creating a new one!");
        // Init a blank slate
        e = nvs_set_blob(this->nvs_handle, NVS_NAME_SS, &this->report_group, s);
        if (e != ESP_OK) {
            ESP_LOGE("SHIFT_REPORTER", "Error writing new Shift report to EEPROM (%s)", esp_err_to_name(e));
        } else {
            failure = false;
        }
    } else if (e != ESP_OK) {
        ESP_LOGE("SHIFT_REPORTER", "Could not read current stored shift report: %s", esp_err_to_name(e));
    } else {
        failure = false;
    }
    // Copy structure over
    if(!failure) {
        ESP_LOGI("SHIFT_REPORTER", "Init OK!");
    }
    memset(&this->report_group, 0x00, sizeof(ShiftReportNvsGroup));
}

ShiftReporter::~ShiftReporter() {
}

void ShiftReporter::add_report(ShiftReport src) {
    // Which index to we write to?
    if (this->report_group.index >= MAX_REPORTS) {
        this->report_group.index = 0;
    }
    uint8_t idx = this->report_group.index;
    ESP_LOGI("SHIFTER", "Writing to report %d as writable", idx);
    this->report_group.reports[idx] = src;
    this->report_group.index++;
    this->has_change = true;
}


void ShiftReporter::save() {
    if (!has_change) {
        return;
    }
    nvs_set_blob(this->nvs_handle, NVS_NAME_SS, &this->report_group, sizeof(ShiftReportNvsGroup));
    nvs_commit(this->nvs_handle);
}