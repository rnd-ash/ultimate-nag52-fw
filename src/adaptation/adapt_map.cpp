
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
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 1->2: Pulling: (%d %d %d), 25%%: (%d %d %d), 50%%: (%d %d %d), 75%%: (%d %d %d)", 
            this->adapt_data[0].trq_neg.spc_fill_adder,
            this->adapt_data[0].trq_neg.mpc_fill_adder,
            this->adapt_data[0].trq_neg.fill_time_adder,
            this->adapt_data[0].trq_25.spc_fill_adder,
            this->adapt_data[0].trq_25.mpc_fill_adder,
            this->adapt_data[0].trq_25.fill_time_adder,
            this->adapt_data[0].trq_50.spc_fill_adder,
            this->adapt_data[0].trq_50.mpc_fill_adder,
            this->adapt_data[0].trq_50.fill_time_adder,
            this->adapt_data[0].trq_75.spc_fill_adder,
            this->adapt_data[0].trq_75.mpc_fill_adder,
            this->adapt_data[0].trq_75.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 2->3: Pulling: (%d %d %d), 25%%: (%d %d %d), 50%%: (%d %d %d), 75%%: (%d %d %d)", 
            this->adapt_data[1].trq_neg.spc_fill_adder,
            this->adapt_data[1].trq_neg.mpc_fill_adder,
            this->adapt_data[1].trq_neg.fill_time_adder,
            this->adapt_data[1].trq_25.spc_fill_adder,
            this->adapt_data[1].trq_25.mpc_fill_adder,
            this->adapt_data[1].trq_25.fill_time_adder,
            this->adapt_data[1].trq_50.spc_fill_adder,
            this->adapt_data[1].trq_50.mpc_fill_adder,
            this->adapt_data[1].trq_50.fill_time_adder,
            this->adapt_data[1].trq_75.spc_fill_adder,
            this->adapt_data[1].trq_75.mpc_fill_adder,
            this->adapt_data[1].trq_75.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 3->4: Pulling: (%d %d %d), 25%%: (%d %d %d), 50%%: (%d %d %d), 75%%: (%d %d %d)", 
            this->adapt_data[2].trq_neg.spc_fill_adder,
            this->adapt_data[2].trq_neg.mpc_fill_adder,
            this->adapt_data[2].trq_neg.fill_time_adder,
            this->adapt_data[2].trq_25.spc_fill_adder,
            this->adapt_data[2].trq_25.mpc_fill_adder,
            this->adapt_data[2].trq_25.fill_time_adder,
            this->adapt_data[2].trq_50.spc_fill_adder,
            this->adapt_data[2].trq_50.mpc_fill_adder,
            this->adapt_data[2].trq_50.fill_time_adder,
            this->adapt_data[2].trq_75.spc_fill_adder,
            this->adapt_data[2].trq_75.mpc_fill_adder,
            this->adapt_data[2].trq_75.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 4->5: Pulling: (%d %d %d), 25%%: (%d %d %d), 50%%: (%d %d %d), 75%%: (%d %d %d)", 
            this->adapt_data[3].trq_neg.spc_fill_adder,
            this->adapt_data[3].trq_neg.mpc_fill_adder,
            this->adapt_data[3].trq_neg.fill_time_adder,
            this->adapt_data[3].trq_25.spc_fill_adder,
            this->adapt_data[3].trq_25.mpc_fill_adder,
            this->adapt_data[3].trq_25.fill_time_adder,
            this->adapt_data[3].trq_50.spc_fill_adder,
            this->adapt_data[3].trq_50.mpc_fill_adder,
            this->adapt_data[3].trq_50.fill_time_adder,
            this->adapt_data[3].trq_75.spc_fill_adder,
            this->adapt_data[3].trq_75.mpc_fill_adder,
            this->adapt_data[3].trq_75.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 2->1: Pulling: (%d %d %d), 25%%: (%d %d %d), 50%%: (%d %d %d), 75%%: (%d %d %d)", 
            this->adapt_data[4].trq_neg.spc_fill_adder,
            this->adapt_data[4].trq_neg.mpc_fill_adder,
            this->adapt_data[4].trq_neg.fill_time_adder,
            this->adapt_data[4].trq_25.spc_fill_adder,
            this->adapt_data[4].trq_25.mpc_fill_adder,
            this->adapt_data[4].trq_25.fill_time_adder,
            this->adapt_data[4].trq_50.spc_fill_adder,
            this->adapt_data[4].trq_50.mpc_fill_adder,
            this->adapt_data[4].trq_50.fill_time_adder,
            this->adapt_data[4].trq_75.spc_fill_adder,
            this->adapt_data[4].trq_75.mpc_fill_adder,
            this->adapt_data[4].trq_75.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 3->2: Pulling: (%d %d %d), 25%%: (%d %d %d), 50%%: (%d %d %d), 75%%: (%d %d %d)", 
            this->adapt_data[5].trq_neg.spc_fill_adder,
            this->adapt_data[5].trq_neg.mpc_fill_adder,
            this->adapt_data[5].trq_neg.fill_time_adder,
            this->adapt_data[5].trq_25.spc_fill_adder,
            this->adapt_data[5].trq_25.mpc_fill_adder,
            this->adapt_data[5].trq_25.fill_time_adder,
            this->adapt_data[5].trq_50.spc_fill_adder,
            this->adapt_data[5].trq_50.mpc_fill_adder,
            this->adapt_data[5].trq_50.fill_time_adder,
            this->adapt_data[5].trq_75.spc_fill_adder,
            this->adapt_data[5].trq_75.mpc_fill_adder,
            this->adapt_data[5].trq_75.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 4->3: Pulling: (%d %d %d), 25%%: (%d %d %d), 50%%: (%d %d %d), 75%%: (%d %d %d)", 
            this->adapt_data[6].trq_neg.spc_fill_adder,
            this->adapt_data[6].trq_neg.mpc_fill_adder,
            this->adapt_data[6].trq_neg.fill_time_adder,
            this->adapt_data[6].trq_25.spc_fill_adder,
            this->adapt_data[6].trq_25.mpc_fill_adder,
            this->adapt_data[6].trq_25.fill_time_adder,
            this->adapt_data[6].trq_50.spc_fill_adder,
            this->adapt_data[6].trq_50.mpc_fill_adder,
            this->adapt_data[6].trq_50.fill_time_adder,
            this->adapt_data[6].trq_75.spc_fill_adder,
            this->adapt_data[6].trq_75.mpc_fill_adder,
            this->adapt_data[6].trq_75.fill_time_adder
        );
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for 5->4: Pulling: (%d %d %d), 25%%: (%d %d %d), 50%%: (%d %d %d), 75%%: (%d %d %d)", 
            this->adapt_data[7].trq_neg.spc_fill_adder,
            this->adapt_data[7].trq_neg.mpc_fill_adder,
            this->adapt_data[7].trq_neg.fill_time_adder,
            this->adapt_data[7].trq_25.spc_fill_adder,
            this->adapt_data[7].trq_25.mpc_fill_adder,
            this->adapt_data[7].trq_25.fill_time_adder,
            this->adapt_data[7].trq_50.spc_fill_adder,
            this->adapt_data[7].trq_50.mpc_fill_adder,
            this->adapt_data[7].trq_50.fill_time_adder,
            this->adapt_data[7].trq_75.spc_fill_adder,
            this->adapt_data[7].trq_75.mpc_fill_adder,
            this->adapt_data[7].trq_75.fill_time_adder
        );
    }
}

void AdaptationMap::reset() {
    for (int i = 0; i < 8; i++) {
        this->adapt_data[i].trq_neg = DEFAULT_CELL;
        this->adapt_data[i].trq_25 = DEFAULT_CELL;
        this->adapt_data[i].trq_50 = DEFAULT_CELL;
        this->adapt_data[i].trq_75 = DEFAULT_CELL;
    }
    ESP_LOGI("ADAPT_MAP", "Adaptation reset!");
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



const AdaptationCell* AdaptationMap::get_adapt_cell(SensorData* sensors, ProfileGearChange change, uint16_t gb_max_torque) {
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

    uint16_t quarter = (uint16_t)((float)gb_max_torque/4.0);
    if (sensors->static_torque <= 0) {
        return &this->adapt_data[adaptation_idx].trq_neg;
    } else if (sensors->static_torque <= quarter) {
        return &this->adapt_data[adaptation_idx].trq_25;
    } else if (sensors->static_torque <= quarter*2) {
        return &this->adapt_data[adaptation_idx].trq_50;
    } else {
        return &this->adapt_data[adaptation_idx].trq_75; 
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

    float q_trq =  (float)gb_max_torque/4.0;
    if (sensors->static_torque <= 0) {
        dest = &this->adapt_data[adaptation_idx].trq_neg;
    } else if (sensors->static_torque <= q_trq) {
        dest = &this->adapt_data[adaptation_idx].trq_25;
    } else if (sensors->static_torque <= q_trq*2) {
        dest = &this->adapt_data[adaptation_idx].trq_25;
    } else if (sensors->static_torque <= q_trq*3) {
        dest = &this->adapt_data[adaptation_idx].trq_25;
    } else {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "ADAPT_MAP", "Cannot adapt. Torque outside limit. Got %d Nm, Max allowed: %d Nm",
            sensors->static_torque,
            (uint16_t)q_trq*3
        );
        return;
    }

    // Calc when the transitions started
    int start_hold3 = rpt->hold1_data.hold_time+rpt->hold1_data.ramp_time + rpt->hold2_data.hold_time+rpt->hold2_data.ramp_time;
    int start_overlap = start_hold3 + rpt->hold3_data.hold_time+rpt->hold3_data.ramp_time;
    int start_max_p   = start_overlap + rpt->overlap_data.hold_time+rpt->overlap_data.ramp_time;

    int shift_time = rpt->transition_end - rpt->transition_start;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Shift took %d ms", shift_time);
    if (rpt->transition_start < start_hold3) {
        ESP_LOGW("ADAPT_MAP", "Shift started BEFORE hold3!");
        if (rpt->flare_timestamp != 0) {
            dest->mpc_fill_adder += 5;
        } else {
            dest->spc_fill_adder -= 10;
        }
    } else if (rpt->transition_start < start_overlap) {
        ESP_LOGW("ADAPT_MAP", "Shift started BEFORE overlap phase! (%d ms into fill phase)", rpt->transition_start-start_hold3);
    } else if (rpt->transition_start < start_max_p) {
        ESP_LOGW("ADAPT_MAP", "Shift started in overlap phase! (%d ms into overlap phase)", rpt->transition_start-start_overlap);
    } else if (rpt->transition_start > start_max_p) {
        ESP_LOGW("ADAPT_MAP", "Shift started after overlap phase! (%d ms into max pressure phase)", rpt->transition_start-start_max_p);
        dest->mpc_fill_adder -= 10;
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
        dest->spc_fill_adder+=10;
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for target cell are (S %d mBar, M %d mBar) and %d ms", 
            dest->spc_fill_adder,
            dest->mpc_fill_adder,
            dest->fill_time_adder
        );

}