
#include "adaptation/adapt_map.h"
#include <string.h>
#include <esp_log.h>
#include "nvs.h"
#include "nvs/eeprom_config.h"

AdaptationMap::AdaptationMap() {
    if(!this->load_from_nvs()) { // Init our map
        ESP_LOGE("ADAPT_MAP", "Load from NVS failed, starting a new map");
        this->reset();
    } else {
        ESP_LOGI("ADAPT_MAP", "Adapt map loaded!");
        ESP_LOGI("ADAPT_MAP", "Values for 1->2: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[0].offset_accel_idle,
            this->adapt_data[0].bite_speed_accel_idle,
            this->adapt_data[0].offset_accel_load,
            this->adapt_data[0].bite_speed_accel_load,
            this->adapt_data[0].offset_decel_idle,
            this->adapt_data[0].bite_speed_decel_idle,
            this->adapt_data[0].offset_decel_load,
            this->adapt_data[0].bite_speed_decel_load
        );
        ESP_LOGI("ADAPT_MAP", "Values for 2->3: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[1].offset_accel_idle,
            this->adapt_data[1].bite_speed_accel_idle,
            this->adapt_data[1].offset_accel_load,
            this->adapt_data[1].bite_speed_accel_load,
            this->adapt_data[1].offset_decel_idle,
            this->adapt_data[1].bite_speed_decel_idle,
            this->adapt_data[1].offset_decel_load,
            this->adapt_data[1].bite_speed_decel_load
        );
        ESP_LOGI("ADAPT_MAP", "Values for 3->4: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[2].offset_accel_idle,
            this->adapt_data[2].bite_speed_accel_idle,
            this->adapt_data[2].offset_accel_load,
            this->adapt_data[2].bite_speed_accel_load,
            this->adapt_data[2].offset_decel_idle,
            this->adapt_data[2].bite_speed_decel_idle,
            this->adapt_data[2].offset_decel_load,
            this->adapt_data[2].bite_speed_decel_load
        );
        ESP_LOGI("ADAPT_MAP", "Values for 4->5: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[3].offset_accel_idle,
            this->adapt_data[3].bite_speed_accel_idle,
            this->adapt_data[3].offset_accel_load,
            this->adapt_data[3].bite_speed_accel_load,
            this->adapt_data[3].offset_decel_idle,
            this->adapt_data[3].bite_speed_decel_idle,
            this->adapt_data[3].offset_decel_load,
            this->adapt_data[3].bite_speed_decel_load
        );
        ESP_LOGI("ADAPT_MAP", "Values for 2->1: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[4].offset_accel_idle,
            this->adapt_data[4].bite_speed_accel_idle,
            this->adapt_data[4].offset_accel_load,
            this->adapt_data[4].bite_speed_accel_load,
            this->adapt_data[4].offset_decel_idle,
            this->adapt_data[4].bite_speed_decel_idle,
            this->adapt_data[4].offset_decel_load,
            this->adapt_data[4].bite_speed_decel_load
        );
        ESP_LOGI("ADAPT_MAP", "Values for 3->2: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[5].offset_accel_idle,
            this->adapt_data[5].bite_speed_accel_idle,
            this->adapt_data[5].offset_accel_load,
            this->adapt_data[5].bite_speed_accel_load,
            this->adapt_data[5].offset_decel_idle,
            this->adapt_data[5].bite_speed_decel_idle,
            this->adapt_data[5].offset_decel_load,
            this->adapt_data[5].bite_speed_decel_load
        );
        ESP_LOGI("ADAPT_MAP", "Values for 4->3: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[6].offset_accel_idle,
            this->adapt_data[6].bite_speed_accel_idle,
            this->adapt_data[6].offset_accel_load,
            this->adapt_data[6].bite_speed_accel_load,
            this->adapt_data[6].offset_decel_idle,
            this->adapt_data[6].bite_speed_decel_idle,
            this->adapt_data[6].offset_decel_load,
            this->adapt_data[6].bite_speed_decel_load
        );
        ESP_LOGI("ADAPT_MAP", "Values for 5->4: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[7].offset_accel_idle,
            this->adapt_data[7].bite_speed_accel_idle,
            this->adapt_data[7].offset_accel_load,
            this->adapt_data[7].bite_speed_accel_load,
            this->adapt_data[7].offset_decel_idle,
            this->adapt_data[7].bite_speed_decel_idle,
            this->adapt_data[7].offset_decel_load,
            this->adapt_data[7].bite_speed_decel_load
        );
    }
}

void AdaptationMap::reset() {
    for (int i = 0; i < 8; i++) {
        this->adapt_data[i].offset_accel_idle = 0;
        this->adapt_data[i].bite_speed_accel_idle = 1;
        this->adapt_data[i].offset_accel_load = 0;
        this->adapt_data[i].bite_speed_accel_load = 1;
        this->adapt_data[i].offset_decel_idle = 0;
        this->adapt_data[i].bite_speed_decel_idle = 1;
        this->adapt_data[i].offset_decel_load = 0;
        this->adapt_data[i].bite_speed_decel_load = 1;

    }
}

bool AdaptationMap::load_from_nvs() {
    nvs_handle_t config_handle;    
    esp_err_t err = nvs_open("Configuration", NVS_READWRITE, &config_handle);
    if (err != ESP_OK) {
        ESP_LOGE("ADAPT_MAP", "EEPROM NVS handle failed! %s", esp_err_to_name(err));
        return false;
    }
    size_t map_size = sizeof(this->adapt_data);
    err = nvs_get_blob(config_handle, NVS_KEY_GEAR_ADAPTATION, &this->adapt_data, &map_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) { // Not found, need to create a new fresh map
        this->reset(); // Init default
        return this->save();
    } else {
        return (err == ESP_OK);
    }
}

bool AdaptationMap::save() {
    nvs_handle_t handle;
    nvs_open("Configuration", NVS_READWRITE, &handle); // Must succeed as we have already opened it!
    esp_err_t e = nvs_set_blob(handle, NVS_KEY_GEAR_ADAPTATION, &this->adapt_data, sizeof(this->adapt_data));
    if (e != ESP_OK) {
        ESP_LOGE("ADAPT_MAP", "Error setting AdaptMap blob (%s)", esp_err_to_name(e));
        return false;
    }
    e = nvs_commit(handle);
    if (e != ESP_OK) {
        ESP_LOGE("ADAPT_MAP", "Error calling nvs_commit: %s", esp_err_to_name(e));
        return false;
    }
    return true;
}

float AdaptationMap::get_adaptation_speed(SensorData* sensors, ProfileGearChange change) {
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
            return 1.0; // huh??
    }
    if (sensors->static_torque < 0) { // Idle
        if (sensors->d_output_rpm > 0) {
            return this->adapt_data[adaptation_idx].bite_speed_accel_idle;
        } else {
            return this->adapt_data[adaptation_idx].bite_speed_decel_idle;
        }
    } else { // Load
        if (sensors->d_output_rpm > 0) {
            return this->adapt_data[adaptation_idx].bite_speed_accel_load;
        } else {
            return this->adapt_data[adaptation_idx].bite_speed_decel_load;
        }
    }
}

int AdaptationMap::get_adaptation_offset(SensorData* sensors, ProfileGearChange change) {
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
            return 0; // huh??
    }
    if (sensors->static_torque < 0) { // Idle
        if (sensors->d_output_rpm > 0) {
            return this->adapt_data[adaptation_idx].offset_accel_idle;
        } else {
            return this->adapt_data[adaptation_idx].offset_decel_idle;
        }
    } else { // Load
        if (sensors->d_output_rpm > 0) {
            return this->adapt_data[adaptation_idx].offset_accel_load;
        } else {
            return this->adapt_data[adaptation_idx].offset_decel_load;
        }
    }

}

void AdaptationMap::perform_adaptation(SensorData* sensors, ProfileGearChange change, ShiftResponse response, bool upshift) {
    ESP_LOGI("ADAPT_MAP", "Adapting called");
    // Firstly, obey RPM and Torque limits
    /*
    Torque limit no longer needed - EGS will tell engine to always limit torque to a sensible place
    if (sensors->static_torque > ADAPT_TORQUE_LIMIT) {
        ESP_LOGW("ADAPT_MAP", "Cannot adapt. Torque outside limits. Got %d Nm, must be below %d Nm",
            sensors->static_torque,
            ADAPT_TORQUE_LIMIT
        );
        return;
    }
    */
    if (sensors->engine_rpm > ADAPT_RPM_LIMIT) {
        ESP_LOGW("ADAPT_MAP", "Cannot adapt. Engine RPM outside limits. Got %d RPM, must be below %d RPM",
            sensors->engine_rpm,
            ADAPT_RPM_LIMIT
        );
        return;
    }
    // Next check if measure was OK!
    if (!response.measure_ok) {
        ESP_LOGW("ADAPT_MAP", "Cannot adapt. Shift measure was invalid");
        return;
    }

    if (sensors->atf_temp < ADAPT_TEMP_THRESH || sensors->atf_temp > ADAPT_TEMP_LIMIT) {
        ESP_LOGW("ADAPT_MAP", "Cannot adapt. ATF temp outside limits. ATF at %d C, must be between %d and %d C",
            sensors->atf_temp,
            ADAPT_TEMP_THRESH,
            ADAPT_TEMP_LIMIT
        );
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
    ESP_LOGI("ADAPT_MAP", "Adapting...");
    bool accel_shift = sensors->d_output_rpm > 0;
    bool idle_shift = sensors->static_torque > 0;

    if (response.spc_map_start - response.spc_change_start > 20) { // SPC is taking too long to bite so reduce it || Gearbox flared so needs more SPC
        if (idle_shift) {
            if (accel_shift) {
                this->adapt_data[adaptation_idx].offset_accel_idle -= 10; // +1% pressure
            } else {
                this->adapt_data[adaptation_idx].offset_decel_idle -= 10; // +1% pressure
            }
        } else {
            if (accel_shift) {
                this->adapt_data[adaptation_idx].offset_accel_load -= 5; // +1% pressure
            } else {
                this->adapt_data[adaptation_idx].offset_decel_load -= 5; // +1% pressure
            }
        }
    } else if (response.spc_map_start - response.spc_change_start < 10) { // SPC tolorance is too tight, decrease initial SPC pressure
        if (idle_shift) {
            if (accel_shift) {
                this->adapt_data[adaptation_idx].offset_accel_idle += 2; // -0.5% pressure
            } else {
                this->adapt_data[adaptation_idx].offset_decel_idle += 2; // -0.5% pressure
            }
        } else {
            if (accel_shift) {
                this->adapt_data[adaptation_idx].offset_accel_load += 5; // -0.5% pressure
            } else {
                this->adapt_data[adaptation_idx].offset_decel_load += 5; // -0.5% pressure
            }
        }
    }
    ESP_LOGI("ADAPT_MAP", "Values MODIFIED!: AI: (%d, %.2f) AL: (%d, %.2f) DI: (%d, %.2f) DL: (%d, %.2f)", 
            this->adapt_data[adaptation_idx].offset_accel_idle,
            this->adapt_data[adaptation_idx].bite_speed_accel_idle,
            this->adapt_data[adaptation_idx].offset_accel_load,
            this->adapt_data[adaptation_idx].bite_speed_accel_load,
            this->adapt_data[adaptation_idx].offset_decel_idle,
            this->adapt_data[adaptation_idx].bite_speed_decel_idle,
            this->adapt_data[adaptation_idx].offset_decel_load,
            this->adapt_data[adaptation_idx].bite_speed_decel_load
        );
}