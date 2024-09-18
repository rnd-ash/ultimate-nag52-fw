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
            if (CAL_RAM_PTR->magic != 0xDEADBEEFu) {
                // Magic failed
                ret = ESP_ERR_INVALID_VERSION;
                goto exit;
            }
            if (CAL_RAM_PTR->len != sizeof(CalibrationInfo)) {
                // Size mismatch (Maybe data is added?)
                ESP_LOGE("CAL", "Calibration load failed. Length mismatch. Length at info is %d, calibration size is %d", (int)CAL_RAM_PTR->len, (int)sizeof(CalibrationInfo));
                ret = ESP_ERR_INVALID_SIZE;
                goto exit;
            }
            uint16_t crc_calculated = crc(&(reinterpret_cast<uint8_t*>(CAL_RAM_PTR))[8], sizeof(CalibrationInfo)-8);
            if (crc_calculated != CAL_RAM_PTR->crc) {
                // CRC Error
                ESP_LOGE("CAL", "Calibration load failed. CRC error. Wanted %04X, got %04X", crc_calculated, CAL_RAM_PTR->crc);
                ret = ESP_ERR_INVALID_CRC;
                goto exit;
            }
            // Guard against zeroed mechanical CAL
            const uint16_t* r_table = CAL_RAM_PTR->mech_cal.ratio_table;
            if (r_table[1] == 0 || r_table[2] == 0  || r_table[3] == 0  || r_table[4] == 0  || r_table[5] == 0) {
                ESP_LOGE("CAL", "Calibration load failed. Ratio table has 0'ed values!");
                ret = ESP_ERR_INVALID_ARG;
                goto exit;
            }
            // All OK!
            HYDR_PTR = &CAL_RAM_PTR->hydr_cal;
            MECH_PTR = &CAL_RAM_PTR->mech_cal;
            TCC_CFG_PTR = &CAL_RAM_PTR->tcc_cal;
            SHIFT_ALGO_CFG_PTR = &CAL_RAM_PTR->shift_algo_cal;
        }
    }
exit:
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
                // Copy OK!
                if (tmp->magic != 0xDEADBEEFu) {
                    // Magic failed
                    ret = ESP_ERR_INVALID_VERSION;
                    goto exit;
                }
                if (tmp->len != sizeof(CalibrationInfo)) {
                    // Size mismatch (Maybe data is added?)
                    ESP_LOGE("CAL", "Calibration load failed. Length mismatch. Length at info is %d, calibration size is %d", (int)tmp->len, (int)sizeof(CalibrationInfo));
                    ret = ESP_ERR_INVALID_SIZE;
                    goto exit;
                }
                uint16_t crc_calculated = crc(&(reinterpret_cast<uint8_t*>(tmp))[8], sizeof(CalibrationInfo)-8);
                if (crc_calculated != tmp->crc) {
                    // CRC Error
                    ESP_LOGE("CAL", "Calibration load failed. CRC error. Wanted %04X, got %04X", crc_calculated, tmp->crc);
                    ret = ESP_ERR_INVALID_CRC;
                    goto exit;
                }
                // Guard against zeroed mechanical CAL
                const uint16_t* r_table = CAL_RAM_PTR->mech_cal.ratio_table;
                if (r_table[1] == 0 || r_table[2] == 0  || r_table[3] == 0  || r_table[4] == 0  || r_table[5] == 0) {
                    ESP_LOGE("CAL", "Calibration load failed. Ratio table has 0'ed values!");
                    ret = ESP_ERR_INVALID_ARG;
                    goto exit;
                }
                // Copy the temporery CalInfo to the in use one!
                memcpy(CAL_RAM_PTR, tmp, sizeof(CalibrationInfo));
            }
            TCU_FREE(tmp);
        }
    }
exit:
    return ret;
}