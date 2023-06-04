
#include "shift_adaptation.h"
#include <string.h>
#include <esp_log.h>
#include "nvs.h"
#include "nvs/eeprom_config.h"
#include "esp_check.h"
#include "nvs/module_settings.h"

ShiftAdaptationSystem::ShiftAdaptationSystem(void)
{
    
}

/**
 * To do prefill adaptation, we do the following.
 * --- shift_adaptation.cpp ---
 * 1. Check if conditions are correct to try adapting
 * 2. Find which prefill we should adapt (Coasting or pulling)
 * 3. Set the values for override shift characteristics
 * --- Gearbox.cpp --
 * 4. Limit input torque to whats requested in override 
 * 5. Set prefill pressure ShiftAdaptation prefill map
 * 6. Slowy increase prefill pressure by 10mbar/100ms
 * 7. When shift starts, find pressure from ~200ms ago that prefill was at
 * 8. Send this pressure back to adaptation manager to store, along with the user inputs during the shift
 * (NOTE: When doing 'prefill finding', if the user torque demand spikes, we should cancel adapting and shift as normal (SAFETY))
 * (NOTE: 'prefill finding' can only occur if input torque remains fairly stable (+/-10Nm) during the search)
 * --- shift_adaptation.cpp ---
 * 9. Check the user inputs. If they were unstable during the test (IE: User let go of the gas pedal during the prefill find phase),
 *    then invalidate the adaptation results.
 * 10. Check shift delay time
 * 10a. If shift delay < 50ms, then reduce prefill pressure by 100mBar, mark cell as "NOT LEARNED"
 * 10b. If shift delay is between 50ms and 200ms, mark cell as "LEARNED"
 * 10c. If delta between last map prefill and found prefill > 50mbar, mark cell as "NOT LEARNED", and store found prefill as map prefill
 *      (Next shift will use this new value and compare results) 
*/
bool ShiftAdaptationSystem::check_prefill_adapt_request(SensorData* sensors, ProfileGearChange change) {
    return false;

}

esp_err_t ShiftAdaptationSystem::reset(void)
{
    return ESP_OK;
}

esp_err_t ShiftAdaptationSystem::save(void)
{
    return ESP_OK;
}