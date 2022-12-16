
#include "adapt_map.h"
#include <string.h>
#include <esp_log.h>
#include "nvs.h"
#include "nvs/eeprom_config.h"
#include "esp_check.h"

AdaptationMap::AdaptationMap(void)
{
    if (!this->load_from_nvs())
    { // Init our map
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "ADAPT_MAP", "Load from NVS failed, starting a new map");
        this->reset();
    }
    else
    {
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
                      this->adapt_data[0].trq_75.fill_time_adder);
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
                      this->adapt_data[1].trq_75.fill_time_adder);
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
                      this->adapt_data[2].trq_75.fill_time_adder);
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
                      this->adapt_data[3].trq_75.fill_time_adder);
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
                      this->adapt_data[4].trq_75.fill_time_adder);
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
                      this->adapt_data[5].trq_75.fill_time_adder);
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
                      this->adapt_data[6].trq_75.fill_time_adder);
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
                      this->adapt_data[7].trq_75.fill_time_adder);
    }
}

esp_err_t AdaptationMap::reset(void)
{
    for (int i = 0; i < 8; i++)
    {
        this->adapt_data[i].trq_neg = DEFAULT_CELL;
        this->adapt_data[i].trq_25 = DEFAULT_CELL;
        this->adapt_data[i].trq_50 = DEFAULT_CELL;
        this->adapt_data[i].trq_75 = DEFAULT_CELL;
    }
    ESP_LOGI("ADAPT_MAP", "Adaptation reset!");
    return this->save();
}

esp_err_t AdaptationMap::load_from_nvs(void)
{
    esp_err_t result = ESP_OK;
    nvs_handle_t config_handle;
    ESP_RETURN_ON_ERROR(nvs_open("Configuration", NVS_READWRITE, &config_handle), "ADAPT_MAP", "Failed to open NVS");
    size_t map_size = sizeof(this->adapt_data);
    result = nvs_get_blob(config_handle, NVS_KEY_GEAR_ADAPTATION, &this->adapt_data, &map_size);
    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        // Not found, need to create a new fresh map
        this->reset(); // Init default
        result = this->save();
    }
    return result;
}

esp_err_t AdaptationMap::save(void)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open("Configuration", NVS_READWRITE, &handle), "ADAPT_MAP", "Cannot open config handle");
    ESP_RETURN_ON_ERROR(nvs_set_blob(handle, NVS_KEY_GEAR_ADAPTATION, &this->adapt_data, sizeof(this->adapt_data)), "ADAPT_MAP", "Error setting adapt map blob");
    return nvs_commit(handle);
}

inline AdaptationCell *AdaptationMap::get_adapt_cell_from_torque(SensorData *sensors, uint16_t gb_max_torque, uint16_t adaptation_idx, AdaptationMap *adaptationmap_var)
{
    AdaptationCell *result;
    // divide by 4 is equivalent to right shift by 2 on unsigned integer variables
    uint16_t quarter = gb_max_torque >> 2;
    if (0 >= sensors->static_torque)
    {
        result = &adaptationmap_var->adapt_data[adaptation_idx].trq_neg;
    }
    else if (sensors->static_torque <= quarter)
    {
        result = &adaptationmap_var->adapt_data[adaptation_idx].trq_25;
    }
    else if (((uint16_t)sensors->static_torque) <= (quarter * 2u))
    {
        result = &adaptationmap_var->adapt_data[adaptation_idx].trq_50;
    }
    else if (((uint16_t)sensors->static_torque) <= (quarter * 3u))
    {
        result = &adaptationmap_var->adapt_data[adaptation_idx].trq_75;
    }
    else  {
        result = nullptr;
    }
    return result;
}

inline uint16_t AdaptationMap::get_idx_from_change(ProfileGearChange change)
{
    // initialization for invalid index
    uint16_t result = 0xFFFFu;
    switch (change)
    {
    case ProfileGearChange::ONE_TWO:
        result = 0;
        break;
    case ProfileGearChange::TWO_THREE:
        result = 1u;
        break;
    case ProfileGearChange::THREE_FOUR:
        result = 2u;
        break;
    case ProfileGearChange::FOUR_FIVE:
        result = 3u;
        break;
    case ProfileGearChange::TWO_ONE:
        result = 4u;
        break;
    case ProfileGearChange::THREE_TWO:
        result = 5u;
        break;
    case ProfileGearChange::FOUR_THREE:
        result = 6u;
        break;
    case ProfileGearChange::FIVE_FOUR:
        result = 7u;
        break;
    default:
        break;
    }
    return result;
}

AdaptationCell *AdaptationMap::get_adapt_cell(SensorData *sensors, ProfileGearChange change, uint16_t gb_max_torque)
{
    uint16_t adaptation_idx = AdaptationMap::get_idx_from_change(change);
    return (7u >= adaptation_idx) ? this->get_adapt_cell_from_torque(sensors, gb_max_torque, adaptation_idx, this) : const_cast<AdaptationCell *>(&DEFAULT_CELL);
}

void AdaptationMap::perform_adaptation(SensorData *sensors, ShiftReport *rpt, ProfileGearChange change, bool is_valid_rpt, uint16_t gb_max_torque)
{
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Adapting called");
    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT MAP",
                  "Start at %d: \n"
                  "End at %d\n",
                  rpt->transition_start, rpt->transition_end);
    if (is_valid_rpt)
    {
        if (sensors->engine_rpm <= ADAPT_RPM_LIMIT)
        {
            if ((sensors->atf_temp >= ADAPT_TEMP_THRESH) && (sensors->atf_temp <= ADAPT_TEMP_LIMIT))
            {
                AdaptationCell *dest = this->get_adapt_cell(sensors, change, gb_max_torque);
                if (&DEFAULT_CELL != dest)
                {
                    // Calc when the transitions started
                    int start_fill = rpt->bleed_data.hold_time + rpt->bleed_data.ramp_time;
                    int start_overlap = start_fill + rpt->fill_data.hold_time + rpt->fill_data.ramp_time;
                    int start_max_p = start_overlap + rpt->overlap_data.hold_time + rpt->overlap_data.ramp_time;

                    int shift_time = rpt->transition_end - rpt->transition_start;
                    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Shift took %d ms", shift_time);
                    if (rpt->transition_start < start_fill)
                    {
                        ESP_LOGW("ADAPT_MAP", "Shift started BEFORE FILL!");
                        if (rpt->flare_timestamp != 0)
                        {
                            dest->mpc_fill_adder += 20;
                        }
                        else
                        {
                            dest->fill_time_adder += 10;
                        }
                    }
                    else if (rpt->transition_start < start_overlap)
                    {
                        ESP_LOGW("ADAPT_MAP", "Shift started BEFORE overlap phase! (%d ms into fill phase)", rpt->transition_start - start_fill);
                    }
                    else if (rpt->transition_start < start_max_p)
                    {
                        ESP_LOGW("ADAPT_MAP", "Shift started in overlap phase! (%d ms into overlap phase)", rpt->transition_start - start_overlap);
                    }
                    else if (rpt->transition_start > start_max_p)
                    {
                        ESP_LOGW("ADAPT_MAP", "Shift started after overlap phase! (%d ms into max pressure phase)", rpt->transition_start - start_max_p);
                        dest->spc_fill_adder += 20;
                    }
                    else
                    {
                    }
                    if (0 != rpt->flare_timestamp)
                    {
                        if (rpt->flare_timestamp < start_fill)
                        {
                            ESP_LOGW("ADAPT_MAP", "Flare detected BEFORE FILL!");
                        }
                        else if (rpt->flare_timestamp < start_overlap)
                        {
                            ESP_LOGW("ADAPT_MAP", "Flare detected %d ms into fill phase", rpt->flare_timestamp - start_fill);
                        }
                        else if (rpt->flare_timestamp < start_max_p)
                        {
                            ESP_LOGW("ADAPT_MAP", "Flare detected %d ms into overlap phase", rpt->flare_timestamp - start_overlap);
                        }
                        else if (rpt->flare_timestamp > start_max_p)
                        {
                            ESP_LOGW("ADAPT_MAP", "Flare detected %d ms into max pressure phase", rpt->flare_timestamp - start_max_p);
                        }
                        else
                        {
                        }
                    }
                    ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Values for target cell are (S %d mBar, M %d mBar) and %d ms",
                                  dest->spc_fill_adder,
                                  dest->mpc_fill_adder,
                                  dest->fill_time_adder);
                }
                else
                {
                    ESP_LOG_LEVEL(ESP_LOG_WARN, "ADAPT_MAP", "Cannot adapt. Torque outside limit. Got %d Nm, Max allowed: %d Nm",
                                  sensors->static_torque,
                                  ((gb_max_torque >> 2) * 3u));
                }
            }
            else
            {
                // Too cold/hot to adapt
                ESP_LOG_LEVEL(ESP_LOG_WARN, "ADAPT_MAP", "Cannot adapt. ATF temp outside limits. ATF at %d C, must be between %d and %d C",
                              sensors->atf_temp,
                              ADAPT_TEMP_THRESH,
                              ADAPT_TEMP_LIMIT);
            }
        }
        else
        {
            ESP_LOG_LEVEL(ESP_LOG_WARN, "ADAPT_MAP", "Cannot adapt. Engine RPM outside limits. Got %d RPM, must be below %d RPM",
                          sensors->engine_rpm,
                          ADAPT_RPM_LIMIT);
        }
    }
    else
    {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "ADAPT_MAP", "Not adapting as shift was not measurable");
    }
}