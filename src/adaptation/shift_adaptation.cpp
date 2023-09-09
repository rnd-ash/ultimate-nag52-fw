
#include "shift_adaptation.h"
#include <string.h>
#include <esp_log.h>
#include "nvs.h"
#include "nvs/eeprom_config.h"
#include "esp_check.h"
#include "nvs/module_settings.h"
#include "common_structs_ops.h"
#include "maps.h"

const char* CLUTCH_NAMES[5] = {
    "K1",
    "K2",
    "K3",
    "B1",
    "B2",
};

ShiftAdaptationSystem::ShiftAdaptationSystem(GearboxConfiguration* cfg_ptr)
{
    this->gb_cfg = cfg_ptr;
    const int16_t prefill_x_headers[5] = {1,2,3,4,5};
    const int16_t prefill_y_headers[1] = {1};
    this->prefill_pressure_offset_map = new StoredMap(MAP_NAME_PREFILL_ADAPT_PREFILL_PRESSURE, 5*1, prefill_x_headers, prefill_y_headers, 5, 1, PREFILL_ADAPT_PREFILL_PRESSURE_MAP);
    this->prefill_time_offset_map = new StoredMap(MAP_NAME_PREFILL_ADAPT_PREFILL_TIMING, 5*1, prefill_x_headers, prefill_y_headers, 5, 1, PREFILL_ADAPT_PREFILL_TIMING_MAP);
}

esp_err_t ShiftAdaptationSystem::save(void) {
    esp_err_t res_offset = this->prefill_pressure_offset_map->save_to_eeprom();
    esp_err_t res_timing = this->prefill_time_offset_map->save_to_eeprom();
    if (ESP_OK != res_offset) {
        return res_offset;
    } else {
        return res_timing;
    }
}

bool check_prefill_clutch_adapt_allowed(Clutch to_apply) {
    bool ret = false;
    switch(to_apply) {
        case Clutch::K1:
            ret = ADP_CURRENT_SETTINGS.prefill_adapt_k1;
            break;
        case Clutch::K2:
            ret = ADP_CURRENT_SETTINGS.prefill_adapt_k2;
            break;
        case Clutch::K3:
            ret = ADP_CURRENT_SETTINGS.prefill_adapt_k3;
            break;
        case Clutch::B1:
            ret = ADP_CURRENT_SETTINGS.prefill_adapt_b1;
            break;
        case Clutch::B2:
            ret = ADP_CURRENT_SETTINGS.prefill_adapt_b2;
            break;
        case Clutch::B3:
        default:
            break;
    }
    return ret;
}

AdaptPrefillData ShiftAdaptationSystem::get_prefill_adapt_data(Clutch to_apply, Clutch to_release) {
    AdaptPrefillData ret = {0, 0};
    if (nullptr != this->prefill_pressure_offset_map) {
        ret.pressure_offset_on_clutch = this->prefill_pressure_offset_map->get_value((float)to_apply, 1);
        ret.pressure_offset_off_clutch = this->prefill_pressure_offset_map->get_value((float)to_release, 1);
    }
    if (nullptr != this->prefill_time_offset_map) {
        ret.timing_offset = this->prefill_time_offset_map->get_value((float)to_apply, 1);
    }
    this->to_apply = to_apply;
    return ret;
}

void ShiftAdaptationSystem::record_shift_start(uint64_t time_into_shift, int overlap_start_ts, uint16_t mpc, uint16_t spc, ShiftClutchVelocity vel, uint16_t delta_rpm) {
    // Too early, reduce filling time
    // Velocities are calculated every 100ms
    int start_time_offset = ((float)delta_rpm/(float)vel.on_clutch_vel)*100.0;
    int shift_start_time_absolute = (int)time_into_shift - start_time_offset;
    ESP_LOGI("ADAPT", "Estimating shift started -%d ms ago at %d ms", start_time_offset, shift_start_time_absolute);
    bool check_velocity = true;

    if (shift_start_time_absolute < overlap_start_ts) { // Too early in fill phase
        ESP_LOGE("ADAPT", "Shift started too early!");
        this->set_prefill_cell_offset(this->prefill_time_offset_map, this->to_apply, -10.0, 200, -100.0);
    } else if (shift_start_time_absolute > overlap_start_ts + 250) {
        check_velocity = false;
        ESP_LOGE("ADAPT", "Shift started too late!");
        if (this->flared) {
            // There was a flare, TODO - handle MPC padding on the releasing clutch
            ESP_LOGE("ADAPT", "Shift flared prior to overlap, ignoring adaptation");
        } else {
            this->set_prefill_cell_offset(this->prefill_time_offset_map, this->to_apply, +10.0, 200, -100.0);
        }
    }
    if (check_velocity) {
        ESP_LOGI("ADAPT", "Vel on: %d ,Vel off: %d!", vel.on_clutch_vel, vel.off_clutch_vel);
        if (vel.on_clutch_vel > 50) {
            // Too quick
            ESP_LOGW("ADAPT", "Shifting velocity too quick");
            this->set_prefill_cell_offset(this->prefill_pressure_offset_map, this->to_apply, -5.0, 200, -200.0);
        } else if (vel.on_clutch_vel < 20) {
            ESP_LOGW("ADAPT", "Shifting velocity too slow");
            this->set_prefill_cell_offset(this->prefill_pressure_offset_map, this->to_apply, +5.0, 200, -200.0);
        } else {
            ESP_LOGI("ADAPT", "SHIFT PERFECT");
        }
    }
}

void ShiftAdaptationSystem::record_shift_end(ShiftStage c_stage, uint64_t time_into_phase, uint16_t mpc, uint16_t spc) {
    ESP_LOGI("ADAPT", "Shift ended. %d mBar on MPC, %d mBar on SPC", mpc, spc);
}

void ShiftAdaptationSystem::record_flare(ShiftStage when, uint64_t elapsed){
    this->flared = true;
    this->flare_location = when;
    this->flare_time = elapsed;
}

uint16_t ShiftAdaptationSystem::get_overlap_end_shift_pressure(Clutch to_apply, uint16_t selected_prefill_pressure) {
    uint16_t ret = selected_prefill_pressure*2;
    if (nullptr != this->prefill_pressure_offset_map) {
        ret = this->prefill_pressure_offset_map->get_value((float)to_apply, 1);
    }
    return ret;
}

uint32_t ShiftAdaptationSystem::check_prefill_adapt_conditions_start(SensorData* sensors, ProfileGearChange change) {
    uint32_t ret = (int)AdaptCancelFlag::ADAPTABLE;
    this->to_apply = get_clutch_to_apply(change);
    this->flared = false;
    this->flare_location = ShiftStage::Bleed;
    this->flare_time = 0;
    // Check if we can actually adapt this shift
    // 3-2 and 2-1 are not adapted as we can adapt the clutch packs on 3-4 and 4-5 respectively
    if (change == ProfileGearChange::THREE_TWO || change == ProfileGearChange::TWO_ONE) {
        ret |= (uint32_t)AdaptCancelFlag::NOT_ADAPTABLE;
        ESP_LOGI("ADAPT", "Shift is not adaptable");
    } else {
        // Check if user wants to adapt the applying clutch
        Clutch to_apply = get_clutch_to_apply(change);
        if(check_prefill_clutch_adapt_allowed(to_apply)) {
            if (sensors->input_rpm < ADP_CURRENT_SETTINGS.min_input_rpm) {
                ret |= (uint32_t)AdaptCancelFlag::INPUT_RPM_TOO_LOW;
                ESP_LOGI("ADAPT", "Input RPM too low");
            } else if (sensors->input_rpm > ADP_CURRENT_SETTINGS.max_input_rpm) {
                ret |= (uint32_t)AdaptCancelFlag::INPUT_RPM_TOO_HIGH;
                ESP_LOGI("ADAPT", "Input RPM too high");
            }

            if (sensors->atf_temp < ADP_CURRENT_SETTINGS.min_atf_temp) {
                ret |= (uint32_t)AdaptCancelFlag::ATF_TEMP_TOO_LOW;
            } else if (sensors->atf_temp > ADP_CURRENT_SETTINGS.max_atf_temp) {
                ret |= (uint32_t)AdaptCancelFlag::ATF_TEMP_TOO_HIGH;
            }
            if (sensors->input_torque >= gb_cfg->max_torque*0.25) {
                ret |= (uint32_t)AdaptCancelFlag::INPUT_TRQ_TOO_HIGH;
            }
            this->pre_shift_pedal_pos = sensors->pedal_pos;
        } else {
            ret |= (uint32_t)AdaptCancelFlag::USER_CANCELLED;
        }
    }
    return ret;
}

esp_err_t ShiftAdaptationSystem::reset(void)
{
    this->prefill_pressure_offset_map->reset_from_flash();
    this->prefill_time_offset_map->reset_from_flash();
    return ESP_OK;
}

bool ShiftAdaptationSystem::set_prefill_cell_offset(StoredMap* dest, Clutch clutch, int16_t offset, int16_t pos_lim, int16_t neg_lim) {
    bool ret = false;
    if (dest) {
        uint8_t idx = ((uint8_t)clutch) - 1;
        if (idx < dest->get_map_element_count()) {
            int16_t* data = dest->get_current_data();
            int16_t modify = data[idx] + offset;
            if (modify > pos_lim) {
                modify = pos_lim;
            } else if (modify < neg_lim) {
                modify = neg_lim;
            }
            data[idx] = modify;
        }
    }
    return ret;
}

void ShiftAdaptationSystem::debug_print_prefill_data() {
    if (this->prefill_pressure_offset_map && this->prefill_time_offset_map) {
        for (int i = 1; i <= 5; i++) {
            ESP_LOGI("ADAPT", "Prefill data - Clutch %s - (%d mBar, %d ms)", 
                CLUTCH_NAMES[i-1], 
                (int16_t)this->prefill_pressure_offset_map->get_value(i, 1),
                (int16_t)this->prefill_time_offset_map->get_value(i, 1)
            );
        }
    } else {
        ESP_LOGE("ADAPT", "Cannot print prefill data, 1 or more maps is null!");
    }
}