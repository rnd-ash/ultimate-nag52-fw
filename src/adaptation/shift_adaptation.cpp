
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
    const int16_t prefill_y_headers[3] = {0, 10, 25};
    this->prefill_pressure_offset_map = new StoredMap(MAP_NAME_ADAPT_PREFILL_PRESSURE, 5*3, prefill_x_headers, prefill_y_headers, 5, 3, ADAPT_PREFILL_PRESSURE_MAP);
}

esp_err_t ShiftAdaptationSystem::save(void) {
    return this->prefill_pressure_offset_map->save_to_eeprom();
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

int16_t ShiftAdaptationSystem::get_prefill_pressure_offset(int16_t trq_nm, Clutch to_apply) {
    int16_t ret = 0;
    if (nullptr != this->prefill_pressure_offset_map) {
        float trq_percent = ((float)trq_nm*100.0)/(float)(this->gb_cfg->max_torque);
        ret = this->prefill_pressure_offset_map->get_value((float)to_apply, trq_percent);
        ESP_LOGI("ADAPT", "Using prefill adder of %d mBar for %s", ret, CLUTCH_NAMES[(int)to_apply-1]);
    }
    this->to_apply = to_apply;
    return ret;
}

uint32_t ShiftAdaptationSystem::check_prefill_adapt_conditions_start(SensorData* sensors, ProfileGearChange change, int16_t* dest_trq_limit) {
    uint32_t ret = (int)AdaptCancelFlag::ADAPTABLE;
    this->to_apply = get_clutch_to_apply(change);
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
            if (sensors->input_torque >= gb_cfg->max_torque/2) {
                ret |= (uint32_t)AdaptCancelFlag::INPUT_TRQ_TOO_HIGH;
            }
            if (ret == 0) {
                // Adapt can happen! Lets limit the torque, figure out which cell to fill
                if (sensors->input_torque <= 0) {
                    *dest_trq_limit = 0;
                    this->cell_idx_prefill = CELL_ID_NEG_TRQ;
                } else if (sensors->input_torque <= gb_cfg->max_torque*0.25) {
                    *dest_trq_limit = gb_cfg->max_torque*0.1;
                    this->cell_idx_prefill = CELL_ID_10_PST_TRQ;
                } else if (sensors->input_torque <= gb_cfg->max_torque/2) {
                    *dest_trq_limit = gb_cfg->max_torque*0.25;
                    this->cell_idx_prefill = CELL_ID_25_PST_TRQ;
                } else {
                    ret |= (uint32_t)AdaptCancelFlag::INPUT_TRQ_TOO_HIGH;
                }
                this->pre_shift_pedal_pos = sensors->pedal_pos;
                this->flare_notified = false;
                this->last_overlap_check = 0;
            }
        } else {
            ret |= (uint32_t)AdaptCancelFlag::USER_CANCELLED;
        }
    }
    return ret;
}

uint32_t ShiftAdaptationSystem::check_prefill_adapt_conditions_shift(SensorData* sensors, EgsBaseCan* can) {
    uint32_t ret = (int)AdaptCancelFlag::ADAPTABLE;
    
    if (this->requested_trq != 0) {
        //if (abs(sensors->static_torque - this->requested_trq) > 25) {
        //    ret |= AdaptCancelFlag::INPUT_TRQ_TOO_HIGH;
        //}
    }
    if ((int16_t)sensors->pedal_pos - (int16_t)this->pre_shift_pedal_pos > 64) { // ~ > 25% pedal load increase
        ret |= AdaptCancelFlag::PEDAL_DELTA_TOO_HIGH;
    }
    if (can->esp_torque_intervention_active(sensors->current_timestamp_ms, 500)) { // ESP intervention - We cannot adapt as engine is now obeying ESP
        ret |= AdaptCancelFlag::ESP_INTERVENTION;
    }
    return ret;
}

esp_err_t ShiftAdaptationSystem::reset(void)
{
    return ESP_OK;
}

bool ShiftAdaptationSystem::offset_cell_value(StoredMap* map, Clutch clutch, uint8_t load_cell_idx, int16_t offset) {
    bool ret = false;
    // X is clutch, Y is load point
    if (map) {
        uint8_t idx = ((uint8_t)clutch) - 1;
        idx *= 3;
        idx += load_cell_idx;
        // Double safety
        if (idx < this->prefill_pressure_offset_map->get_map_element_count()) {
            int16_t* data = this->prefill_pressure_offset_map->get_current_data();
            int16_t modify = data[idx] + offset;
            if (modify < -200) {
                ESP_LOGW("ADAPT", "Min positive prefill adaptation met");
                modify = -200;
            } else if (modify > 200) {
                ESP_LOGW("ADAPT", "Max positive prefill adaptation met");
                modify = 200;
            }
            data[idx] = modify;
        }
    }
    return ret;
}

void ShiftAdaptationSystem::on_overlap_start(uint64_t timestamp, uint64_t expected_shift_time, int shift_progress_percent) {
    this->flare_notified = false;
    this->last_overlap_check = timestamp;
    this->overlap_start_time = timestamp;
    this->expected_shift_time = expected_shift_time;
    if (shift_progress_percent > 5) {
        ESP_LOGI("ADAPT", "Shift started too early (Start of overlap). Decreasing prefill of %s by 20mBar", CLUTCH_NAMES[(int)to_apply-1]);
        offset_cell_value(this->prefill_pressure_offset_map,  to_apply, this->cell_idx_prefill, -20.0);
        this->last_shift_progress = 5;
    } else {
        this->last_shift_progress = 0;
    }
}

void ShiftAdaptationSystem::do_prefill_overlap_check(
    uint64_t timestamp,
    bool flaring,
    int shift_progress_percent
) {
    // Assuming shift not done
    if (shift_progress_percent <= 0) {
        this->last_overlap_check = timestamp;
        this->last_shift_progress = 0;
    }
    if (timestamp - this->last_overlap_check > 100) {
        if (this->last_shift_progress == 0 && shift_progress_percent != 0) {
            ESP_LOGI("ADAPT", "Shift started %d ms into overlap", (int)(timestamp-overlap_start_time));
            // Shift has now started, lets check the delta and timestamp (Only for longer shifts)
            if (this->expected_shift_time >= 500) {
                // > 25% of overlap time gone by with no shifting
                bool late_shifting = ((timestamp - this->overlap_start_time) > this->expected_shift_time/4);
                if (late_shifting) {
                    ESP_LOGI("ADAPT", "Shift started too late (%d ms in overlap). Increasing prefill for %s by 20mBar", (int)(timestamp - this->overlap_start_time), CLUTCH_NAMES[(int)to_apply-1]);
                    offset_cell_value(this->prefill_pressure_offset_map,  to_apply, this->cell_idx_prefill, 20.0);
                }
                ESP_LOGI("ADAPT", "Shift adapt result: Was too late?: %d", late_shifting);
            }
        }
        this->last_shift_progress = shift_progress_percent;
        this->last_overlap_check = timestamp;
    }
}

void ShiftAdaptationSystem::notify_early_shift(Clutch to_apply) {
    ESP_LOGI("ADAPT", "Reducing pressure for shift by 20mBar");
    offset_cell_value(this->prefill_pressure_offset_map,  to_apply, this->cell_idx_prefill, -20.0);
}

void ShiftAdaptationSystem::debug_print_offset_array() {
    int16_t* data = this->prefill_pressure_offset_map->get_current_data();
    for (int i = 1; i <= 5; i++) {
        int16_t* ptr = &data[(i-1)*3];
        int p0 = ptr[0];
        int p1 = ptr[1];
        int p2 = ptr[2];
        ESP_LOGI("ADAPT", "Clutch %s - 0%%: %d 10%%: %d 25%%: %d", CLUTCH_NAMES[i-1], p0, p1, p2);
    }
}