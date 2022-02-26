
#include "adaptation/adapt_map.h"
#include <string.h>
#include <esp_log.h>

AdaptationMap::AdaptationMap() {
    if(!this->load_from_nvs()) { // Init our map
        ESP_LOGE("ADAPT_MAP", "Load from NVS failed, starting a new map");
        this->reset();
    } 
}

void AdaptationMap::reset() {
    memset(this->adapt_data, 0x00, sizeof(this->adapt_data));
}

bool load_from_nvs() {
    return false;
}

void save_to_nvs() {
    
}

void AdaptationMap::perform_adaptation(SensorData* sensors, ProfileGearChange change, ShiftResponse response) {
    // Firstly, obey RPM and Torque limits
    if (sensors->engine_rpm > ADAPT_RPM_LIMIT || sensors->static_torque > ADAPT_TORQUE_LIMIT) {
        return;
    }
    // Next check if measure was OK!
    if (!response.measure_ok) {
        return;
    }

    if (sensors->atf_temp < ADAPT_TEMP_THRESH || sensors->atf_temp > ADAPT_TEMP_LIMIT) {
        return; // Too cold/hot to adapt
    }

    int adaptation_idx = 0;
    switch(change) {
        case ProfileGearChange::ONE_TWO:
            adaptation_idx = 0;
            break;
        case ProfileGearChange::TWO_THREE:
            adaptation_idx = 1;
            break;
        case ProfileGearChange::THREE_FOUR:
            adaptation_idx = 2;
            break;
        case ProfileGearChange::FOUR_FIVE:
            adaptation_idx = 3;
            break;
        case ProfileGearChange::TWO_ONE:
            adaptation_idx = 4;
            break;
        case ProfileGearChange::THREE_TWO:
            adaptation_idx = 5;
            break;
        case ProfileGearChange::FOUR_THREE:
            adaptation_idx = 6;
            break;
        case ProfileGearChange::FIVE_FOUR:
            adaptation_idx = 7;
            break;
        default:
            return; // huh??
    }

    bool accel_shift = response.d_output_rpm > 0;
    bool idle_shift = sensors->static_torque > 0;
    if (response.flared) { // Flaring so we reduce initial bite pressure
        if (idle_shift) {
            if (accel_shift) {
                this->adapt_data[adaptation_idx].offset_accel_idle -= 4;
            } else {
                this->adapt_data[adaptation_idx].offset_decel_idle -= 4;
            }
        } else {
            if (accel_shift) {
                this->adapt_data[adaptation_idx].offset_accel_load -= 8;
            } else {
                this->adapt_data[adaptation_idx].offset_decel_load -= 8;
            }
        }
    } else if (response.shift_time_ms < response.targ_shift_time_ms*0.75) { // Shift was too quick, we should back off the pressure a bit
        if (idle_shift) {
            if (accel_shift) {
                this->adapt_data[adaptation_idx].offset_accel_idle += 2;
            } else {
                this->adapt_data[adaptation_idx].offset_decel_idle += 2;
            }
        } else {
            if (accel_shift) {
                this->adapt_data[adaptation_idx].offset_accel_load += 4;
            } else {
                this->adapt_data[adaptation_idx].offset_decel_load += 4;
            }
        }
    }
}