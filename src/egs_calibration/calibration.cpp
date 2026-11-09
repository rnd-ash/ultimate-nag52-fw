#include "calibration_structs.h"
#include "tcu_alloc.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "string.h"

CalibrationInfo* CAL_RAM_PTR = NULL;
HydraulicCalibration* HYDR_PTR = NULL;
MechanicalCalibration* MECH_PTR = NULL;
TorqueConverterCalibration* TCC_CFG_PTR = NULL;
ShiftAlgorithmPack* SHIFT_ALGO_CFG_PTR = NULL;

uint16_t crc(const uint8_t* buffer, uint16_t len) {
    uint16_t res = 0;
    for(uint16_t i = 0; i < len; i++) {
        res += i;
        res += buffer[i];
    }
    return res;
}

// Validate the block that was just read before exposing it to control code.
static esp_err_t validate_calibration(const CalibrationInfo* calibration) {
    if (calibration->magic != 0xDEADBEEFu) {
        return ESP_ERR_INVALID_VERSION;
    }
    if (calibration->len != sizeof(CalibrationInfo)) {
        ESP_LOGE("CAL", "Calibration length mismatch: stored %d, expected %d",
                 (int)calibration->len, (int)sizeof(CalibrationInfo));
        return ESP_ERR_INVALID_SIZE;
    }
    const uint16_t calculated = crc(reinterpret_cast<const uint8_t*>(calibration) + 8,
                                    sizeof(CalibrationInfo) - 8);
    if (calculated != calibration->crc) {
        ESP_LOGE("CAL", "Calibration checksum mismatch: expected %04X, got %04X",
                 calculated, calibration->crc);
        return ESP_ERR_INVALID_CRC;
    }
    // Index packed fields directly rather than exporting an unaligned uint16_t pointer.
    for (int gear = 1; gear <= 5; ++gear) {
        if (calibration->mech_cal.ratio_table[gear] == 0) {
            ESP_LOGE("CAL", "Calibration ratio table has a zero forward ratio");
            return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

esp_err_t EGSCal::init_egs_calibration() {
    CAL_RAM_PTR = reinterpret_cast<CalibrationInfo*>(TCU_HEAP_ALLOC(sizeof(CalibrationInfo)));
    if (CAL_RAM_PTR == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t ret = esp_flash_read(nullptr, CAL_RAM_PTR, CALIBRATION_START_ADDRESS, sizeof(CalibrationInfo));
    if (ret == ESP_OK) {
        ret = validate_calibration(CAL_RAM_PTR);
    }
    if (ret == ESP_OK) {
        HYDR_PTR = &CAL_RAM_PTR->hydr_cal;
        MECH_PTR = &CAL_RAM_PTR->mech_cal;
        TCC_CFG_PTR = &CAL_RAM_PTR->tcc_cal;
        SHIFT_ALGO_CFG_PTR = &CAL_RAM_PTR->shift_algo_cal;
    }
    return ret;
}

esp_err_t EGSCal::reload_egs_calibration() {
    if (CAL_RAM_PTR == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    CalibrationInfo* tmp = reinterpret_cast<CalibrationInfo*>(TCU_HEAP_ALLOC(sizeof(CalibrationInfo)));
    if (tmp == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t ret = esp_flash_read(nullptr, tmp, CALIBRATION_START_ADDRESS, sizeof(CalibrationInfo));
    if (ret == ESP_OK) {
        ret = validate_calibration(tmp);
    }
    if (ret == ESP_OK) {
        memcpy(CAL_RAM_PTR, tmp, sizeof(CalibrationInfo));
    }
    TCU_FREE(tmp);
    return ret;
}
