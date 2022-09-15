
#include "adaptation/adapt_map.h"
#include <string.h>
#include <esp_log.h>
#include "nvs.h"
#include "nvs/eeprom_config.h"

AdaptationMap::AdaptationMap() {
    if(!this->load_from_nvs()) { // Init our map
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "ADAPT_MAP", "Load from NVS failed, starting a new map");
        this->reset();
    } else {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Adapt map loaded!");
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 1->2: AI: (%d mBar, %d ms) AL: (%d mBar, %d ms) DI: (%d mBar, %d ms) DL: (%d mBar, %d ms)", 
            this->adapt_data[0].accel_idle.fill_pressure_mbar,
            this->adapt_data[0].accel_idle.fill_time_adder,
            this->adapt_data[0].accel_load.fill_pressure_mbar,
            this->adapt_data[0].accel_load.fill_time_adder,
            this->adapt_data[0].decel_idle.fill_pressure_mbar,
            this->adapt_data[0].decel_idle.fill_time_adder,
            this->adapt_data[0].decel_load.fill_pressure_mbar,
            this->adapt_data[0].decel_load.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 2->3: AI: (%d mBar, %d ms) AL: (%d mBar, %d ms) DI: (%d mBar, %d ms) DL: (%d mBar, %d ms)", 
            this->adapt_data[1].accel_idle.fill_pressure_mbar,
            this->adapt_data[1].accel_idle.fill_time_adder,
            this->adapt_data[1].accel_load.fill_pressure_mbar,
            this->adapt_data[1].accel_load.fill_time_adder,
            this->adapt_data[1].decel_idle.fill_pressure_mbar,
            this->adapt_data[1].decel_idle.fill_time_adder,
            this->adapt_data[1].decel_load.fill_pressure_mbar,
            this->adapt_data[1].decel_load.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 3->4: AI: (%d mBar, %d ms) AL: (%d mBar, %d ms) DI: (%d mBar, %d ms) DL: (%d mBar, %d ms)", 
            this->adapt_data[2].accel_idle.fill_pressure_mbar,
            this->adapt_data[2].accel_idle.fill_time_adder,
            this->adapt_data[2].accel_load.fill_pressure_mbar,
            this->adapt_data[2].accel_load.fill_time_adder,
            this->adapt_data[2].decel_idle.fill_pressure_mbar,
            this->adapt_data[2].decel_idle.fill_time_adder,
            this->adapt_data[2].decel_load.fill_pressure_mbar,
            this->adapt_data[2].decel_load.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 4->5: AI: (%d mBar, %d ms) AL: (%d mBar, %d ms) DI: (%d mBar, %d ms) DL: (%d mBar, %d ms)", 
            this->adapt_data[3].accel_idle.fill_pressure_mbar,
            this->adapt_data[3].accel_idle.fill_time_adder,
            this->adapt_data[3].accel_load.fill_pressure_mbar,
            this->adapt_data[3].accel_load.fill_time_adder,
            this->adapt_data[3].decel_idle.fill_pressure_mbar,
            this->adapt_data[3].decel_idle.fill_time_adder,
            this->adapt_data[3].decel_load.fill_pressure_mbar,
            this->adapt_data[3].decel_load.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 2->1: AI: (%d mBar, %d ms) AL: (%d mBar, %d ms) DI: (%d mBar, %d ms) DL: (%d mBar, %d ms)", 
            this->adapt_data[4].accel_idle.fill_pressure_mbar,
            this->adapt_data[4].accel_idle.fill_time_adder,
            this->adapt_data[4].accel_load.fill_pressure_mbar,
            this->adapt_data[4].accel_load.fill_time_adder,
            this->adapt_data[4].decel_idle.fill_pressure_mbar,
            this->adapt_data[4].decel_idle.fill_time_adder,
            this->adapt_data[4].decel_load.fill_pressure_mbar,
            this->adapt_data[4].decel_load.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 3->2: AI: (%d mBar, %d ms) AL: (%d mBar, %d ms) DI: (%d mBar, %d ms) DL: (%d mBar, %d ms)", 
            this->adapt_data[5].accel_idle.fill_pressure_mbar,
            this->adapt_data[5].accel_idle.fill_time_adder,
            this->adapt_data[5].accel_load.fill_pressure_mbar,
            this->adapt_data[5].accel_load.fill_time_adder,
            this->adapt_data[5].decel_idle.fill_pressure_mbar,
            this->adapt_data[5].decel_idle.fill_time_adder,
            this->adapt_data[5].decel_load.fill_pressure_mbar,
            this->adapt_data[5].decel_load.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 4->3: AI: (%d mBar, %d ms) AL: (%d mBar, %d ms) DI: (%d mBar, %d ms) DL: (%d mBar, %d ms)", 
            this->adapt_data[6].accel_idle.fill_pressure_mbar,
            this->adapt_data[6].accel_idle.fill_time_adder,
            this->adapt_data[6].accel_load.fill_pressure_mbar,
            this->adapt_data[6].accel_load.fill_time_adder,
            this->adapt_data[6].decel_idle.fill_pressure_mbar,
            this->adapt_data[6].decel_idle.fill_time_adder,
            this->adapt_data[6].decel_load.fill_pressure_mbar,
            this->adapt_data[6].decel_load.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 5->4: AI: (%d mBar, %d ms) AL: (%d mBar, %d ms) DI: (%d mBar, %d ms) DL: (%d mBar, %d ms)", 
            this->adapt_data[7].accel_idle.fill_pressure_mbar,
            this->adapt_data[7].accel_idle.fill_time_adder,
            this->adapt_data[7].accel_load.fill_pressure_mbar,
            this->adapt_data[7].accel_load.fill_time_adder,
            this->adapt_data[7].decel_idle.fill_pressure_mbar,
            this->adapt_data[7].decel_idle.fill_time_adder,
            this->adapt_data[7].decel_load.fill_pressure_mbar,
            this->adapt_data[7].decel_load.fill_time_adder
        );
    }
}

void AdaptationMap::reset() {
    for (int i = 0; i < 8; i++) {
        this->adapt_data[i].accel_idle = DEFAULT_CELL;
        this->adapt_data[i].accel_load = DEFAULT_CELL;
        this->adapt_data[i].decel_idle = DEFAULT_CELL;
        this->adapt_data[i].decel_load = DEFAULT_CELL;
    }
    this->save();
}

bool AdaptationMap::load_from_nvs() {
    nvs_handle_t config_handle;    
    esp_err_t err = nvs_open("Configuration", NVS_READWRITE, &config_handle);
    if (err != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "ADAPT_MAP", "EEPROM NVS handle failed! %s", esp_err_to_name(err));
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
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "ADAPT_MAP", "Error setting AdaptMap blob (%s)", esp_err_to_name(e));
        return false;
    }
    e = nvs_commit(handle);
    if (e != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "ADAPT_MAP", "Error calling nvs_commit: %s", esp_err_to_name(e));
        return false;
    }
    return true;
}



const AdaptationCell* AdaptationMap::get_adapt_cell(SensorData* sensors, ProfileGearChange change) {
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
            return &DEFAULT_CELL; // huh invalid index??
    }
    if (sensors->static_torque < 0) { // Idle
        if (sensors->d_output_rpm > 0) {
            return &this->adapt_data[adaptation_idx].accel_idle;
        } else {
            return &this->adapt_data[adaptation_idx].decel_idle;
        }
    } else { // Load
        if (sensors->d_output_rpm > 0) {
            return &this->adapt_data[adaptation_idx].accel_load;
        } else {
            return &this->adapt_data[adaptation_idx].decel_load;
        }
    }
}

void AdaptationMap::perform_adaptation(SensorData* sensors, ShiftReport* rpt, ProfileGearChange change, bool is_valid_rpt, uint16_t gb_max_torque) {
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Adapting called");
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT MAP", 
    "Start at %d: \n"
    "End at %d\n", 
    rpt->transition_start, rpt->transition_end);
    if (!is_valid_rpt) {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP","Not adapting as shift was not measurable");
        return;
    }

    // Firstly, obey RPM and Torque limits
    if (sensors->static_torque > gb_max_torque/2) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "ADAPT_MAP", "Cannot adapt. Torque outside limits. Got %d Nm, must be below %d Nm",
            sensors->static_torque,
            gb_max_torque/2
        );
        return;
    }
    if (sensors->engine_rpm > ADAPT_RPM_LIMIT) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "ADAPT_MAP", "Cannot adapt. Engine RPM outside limits. Got %d RPM, must be below %d RPM",
            sensors->engine_rpm,
            ADAPT_RPM_LIMIT
        );
        return;
    }

    if (sensors->atf_temp < ADAPT_TEMP_THRESH || sensors->atf_temp > ADAPT_TEMP_LIMIT) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "ADAPT_MAP", "Cannot adapt. ATF temp outside limits. ATF at %d C, must be between %d and %d C",
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
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Adapting...");
    bool accel_shift = sensors->d_output_rpm > 0;
    bool idle_shift = sensors->static_torque > 0;
    AdaptationCell* dest;

    if (accel_shift) {
        if (idle_shift) {
            dest = &this->adapt_data[adaptation_idx].accel_idle;
        } else {
            dest = &this->adapt_data[adaptation_idx].accel_load;
        }
    } else {
        if (idle_shift) {
            dest = &this->adapt_data[adaptation_idx].decel_idle;
        } else {
            dest = &this->adapt_data[adaptation_idx].decel_load;
        }
    }

    // Calc when the transitions started
    int start_hold3 = rpt->hold1_data.hold_time+rpt->hold1_data.ramp_time + rpt->hold2_data.hold_time+rpt->hold2_data.ramp_time;
    int start_overlap = start_hold3 + rpt->hold3_data.hold_time+rpt->hold3_data.ramp_time;
    int start_max_p   = start_overlap + rpt->overlap_data.hold_time+rpt->overlap_data.ramp_time;

    int shift_time = rpt->transition_end - rpt->transition_start;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Shift took %d ms", shift_time);
    if (rpt->transition_start < start_hold3) {
        ESP_LOGW("ADAPT_MAP", "Shift started BEFORE hold3!");
        dest->fill_pressure_mbar -= 10;
    } else if (rpt->transition_start < start_overlap) {
        ESP_LOGW("ADAPT_MAP", "Shift started BEFORE overlap phase! (%d ms into fill phase)", rpt->transition_start-start_hold3);
    } else if (rpt->transition_start < start_max_p) {
        ESP_LOGW("ADAPT_MAP", "Shift started in overlap phase! (%d ms into overlap phase)", rpt->transition_start-start_overlap);
    } else if (rpt->transition_start > start_max_p) {
        ESP_LOGW("ADAPT_MAP", "Shift started after overlap phase! (%d ms into max pressure phase)", rpt->transition_start-start_max_p);
        dest->fill_pressure_mbar += 10;
        dest->fill_time_adder += 20;
    }

    if (rpt->flare_timestamp != 0) {
        if (rpt->flare_timestamp < start_hold3) {
            ESP_LOGW("ADAPT_MAP", "Flare detected BEFORE hold3!");
        } else if (rpt->flare_timestamp < start_overlap) {
            ESP_LOGW("ADAPT_MAP", "Flare detected %d ms into fill phase", rpt->flare_timestamp-start_hold3);
        } else if (rpt->flare_timestamp < start_max_p) {
            ESP_LOGW("ADAPT_MAP", "Flare detected %d ms into overlap phase", rpt->flare_timestamp-start_overlap);
        } else if (rpt->flare_timestamp > start_max_p) {
            ESP_LOGW("ADAPT_MAP", "Flare detected %d ms into max pressure phase", rpt->flare_timestamp-start_max_p);
        }
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for target cell are %d mBar and %d ms", 
            dest->fill_pressure_mbar,
            dest->fill_time_adder
        );

}