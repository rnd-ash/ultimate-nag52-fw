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

namespace {

/**
 * @brief Validates a calibration blob that has just been read out of flash.
 *
 * Shared by the initial load and the hot reload so the two cannot drift apart -
 * the reload used to duplicate these checks and validated the wrong buffer.
 *
 * @param cal The freshly read calibration to check (NOT the one in use)
 */
esp_err_t validate_calibration(const CalibrationInfo* cal) {
    esp_err_t ret = ESP_OK;
    if (nullptr == cal) {
        ret = ESP_ERR_INVALID_ARG;
    } else if (cal->magic != 0xDEADBEEFu) {
        // Magic failed
        ret = ESP_ERR_INVALID_VERSION;
    } else if (cal->len != sizeof(CalibrationInfo)) {
        // Size mismatch (Maybe data is added?)
        ESP_LOGE("CAL", "Calibration load failed. Length mismatch. Length at info is %d, calibration size is %d", (int)cal->len, (int)sizeof(CalibrationInfo));
        ret = ESP_ERR_INVALID_SIZE;
    } else {
        const uint16_t crc_calculated = crc(&(reinterpret_cast<const uint8_t*>(cal))[8], sizeof(CalibrationInfo)-8);
        if (crc_calculated != cal->crc) {
            // CRC Error
            ESP_LOGE("CAL", "Calibration load failed. CRC error. Wanted %04X, got %04X", crc_calculated, cal->crc);
            ret = ESP_ERR_INVALID_CRC;
        } else {
            // Guard against zeroed mechanical CAL.
            // Indexed through the struct rather than via a uint16_t*:
            // MechanicalCalibration is packed and ratio_table lands on an odd
            // offset, so a bare pointer would be dereferenced as if aligned.
            if (cal->mech_cal.ratio_table[1] == 0 ||
                cal->mech_cal.ratio_table[2] == 0 ||
                cal->mech_cal.ratio_table[3] == 0 ||
                cal->mech_cal.ratio_table[4] == 0 ||
                cal->mech_cal.ratio_table[5] == 0) {
                ESP_LOGE("CAL", "Calibration load failed. Ratio table has 0'ed values!");
                ret = ESP_ERR_INVALID_ARG;
            }
        }
    }
    return ret;
}

/**
 * @brief Points the per section accessors at the calibration held in CAL_RAM_PTR
 */
void publish_calibration_sections(void) {
    HYDR_PTR = &CAL_RAM_PTR->hydr_cal;
    MECH_PTR = &CAL_RAM_PTR->mech_cal;
    TCC_CFG_PTR = &CAL_RAM_PTR->tcc_cal;
    SHIFT_ALGO_CFG_PTR = &CAL_RAM_PTR->shift_algo_cal;
}

} // namespace

esp_err_t EGSCal::init_egs_calibration() {
    // First of all, allocate our buffer
    esp_err_t ret = ESP_OK;
    CAL_RAM_PTR = reinterpret_cast<CalibrationInfo*>(TCU_HEAP_ALLOC(sizeof(CalibrationInfo)));
    if (NULL == CAL_RAM_PTR) {
        ret = ESP_ERR_NO_MEM;
    } else {
        // Allocation OK!
        ret = esp_flash_read(NULL, static_cast<void*>(CAL_RAM_PTR), CALIBRATION_START_ADDRESS, sizeof(CalibrationInfo));
        if (ESP_OK == ret) {
            // Copy OK!
            ret = validate_calibration(CAL_RAM_PTR);
            if (ESP_OK == ret) {
                // All OK!
                publish_calibration_sections();
            }
        }
    }
    return ret;
}

esp_err_t EGSCal::reload_egs_calibration() {
    // First of all, allocate a temp buffer
    esp_err_t ret = ESP_OK;
    if (nullptr == CAL_RAM_PTR) { // CAL Info is not set, cannot 'reload', as it was never allocated
        ret = ESP_ERR_INVALID_STATE;
    } else {
        CalibrationInfo* tmp = reinterpret_cast<CalibrationInfo*>(TCU_HEAP_ALLOC(sizeof(CalibrationInfo)));
        if (NULL == tmp) {
            ret = ESP_ERR_NO_MEM;
        } else {
            // Allocation OK!
            ret = esp_flash_read(NULL, static_cast<void*>(tmp), CALIBRATION_START_ADDRESS, sizeof(CalibrationInfo));
            if (ESP_OK == ret) {
                // Copy OK! Validate the buffer we just read, NOT the one in use
                ret = validate_calibration(tmp);
            }
            if (ESP_OK == ret) {
                // Copy the temporery CalInfo to the in use one!
                memcpy(CAL_RAM_PTR, tmp, sizeof(CalibrationInfo));
                // Re-publish: the section pointers are still NULL if the initial
                // load failed, and this is the path that recovers from that.
                publish_calibration_sections();
            }
            // Always reached - the old early returns leaked tmp on every
            // validation failure, and the hot reload routine is diag triggerable.
            TCU_FREE(tmp);
        }
    }
    return ret;
}