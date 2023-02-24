
#include "shift_adaptation.h"
#include <string.h>
#include <esp_log.h>
#include "nvs.h"
#include "nvs/eeprom_config.h"
#include "esp_check.h"

ShiftAdaptationSystem::ShiftAdaptationSystem(void)
{
    
}

esp_err_t ShiftAdaptationSystem::reset(void)
{
    return ESP_OK;
}

esp_err_t ShiftAdaptationSystem::save(void)
{
    return ESP_OK;
}