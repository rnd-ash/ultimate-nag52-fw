#include "hfm_engine.hpp"
#include <esp_log.h>
#include "nvs/all_keys.h"
#include <nvs/eeprom_config.h>

HfmEngine::HfmEngine()
{
    ESP_LOGI(name, "Engine model initialization started");
    // loading maps
    hfm_engine_max_torque = new StoredTable(NVS_KEY_MAP_NAME_HFM_ENGINE_TORQUE, HFM_ENGINE_TABLE_SIZE, ENGINE_RPM_HEADERS, HFM_ENGINE_TABLE_SIZE, HFM_ENGINE_MAX_TORQUE_MAP);
    
    const char* key_name_maf = NVS_KEY_MAP_NAME_HFM_ENGINE_MASS_AIR_FLOW;
    hfm_engine_mass_air_flow = new StoredMap(key_name_maf, HFM_ENGINE_MAP_SIZE, ENGINE_RPM_HEADERS, ENGINE_THROTTLE_HEADERS, HFM_ENGINE_TABLE_SIZE, HFM_ENGINE_TABLE_SIZE, HFM_ENGINE_MASS_AIR_FLOW);
    
    const char* key_name_max_maf = NVS_KEY_MAP_NAME_HFM_ENGINE_MAX_MASS_AIR_FLOW;
    hfm_engine_max_mass_air_flow = new StoredTable(key_name_max_maf, HFM_ENGINE_TABLE_SIZE, ENGINE_RPM_HEADERS, HFM_ENGINE_TABLE_SIZE, HFM_ENGINE_MAX_MASS_AIR_FLOW);    
    if(print_data){
        print();
    }
}

HfmEngine::~HfmEngine()
{
}

int16_t HfmEngine::get_max_torque(uint16_t n_mot)
{
    return (int16_t)(hfm_engine_max_torque->get_value((float)n_mot));
}

float HfmEngine::get_mass_air_flow(uint16_t n_mot, uint8_t throttle)
{
    return (int16_t)(hfm_engine_mass_air_flow->get_value((float)n_mot, (float)throttle));
}

float HfmEngine::get_max_mass_air_flow(uint16_t n_mot)
{
    return hfm_engine_max_mass_air_flow->get_value(n_mot);
}

bool HfmEngine::add_to_mass_air_flow_map(uint16_t n_mot, uint8_t throttle, float mass_air_flow)
{
    const float threshold = 0.0F; // TODO: make this configurable

    if(throttle == VEHICLE_CONFIG.throttlevalve_maxopeningangle){
        // at full throttle, indicated torque is equal to maximum torque
        hfm_engine_max_mass_air_flow->add_value((int16_t)mass_air_flow, (int16_t)n_mot, threshold);
    }
    return hfm_engine_mass_air_flow->add_value((int16_t)mass_air_flow, (int16_t)n_mot, (int16_t)throttle, threshold);
}

float HfmEngine::get_ML(float mle, int16_t iat, int16_t air_pressure)
{
    // mass volumetric efficiency
    float lambda_l = 0.0f;
    // conversion to [kg/s]
    float m_Air = mle / 3600.F;
    float m_theoretical = 0.0f; // TODO:
    // engine displacement [m³]
    float engine_displacement = ((float)V_H) * 0.001f;
    // air density [kg/m³]
    float rho_theoretical = 1.188f;
    // temperature [K]
    float T_theoretical = (float)iat + 273.15f;
    // pressure [N/m²]
    float p_theoretical = ((float)air_pressure) * 100.F; // conversion from [hPa] to [Pa]
    rho_theoretical = T_theoretical / p_theoretical;

    m_theoretical = rho_theoretical * engine_displacement / ((float)n_cylinders);
    if (0.0f != m_theoretical)
    {
        lambda_l = m_Air / m_theoretical;
    }
    return lambda_l;
}

void HfmEngine::load(void)
{
    if(hfm_engine_mass_air_flow) {
        hfm_engine_mass_air_flow->reset_from_flash();
    }
    if(hfm_engine_max_mass_air_flow) {
        hfm_engine_max_mass_air_flow->reset_from_flash();
    }
    if(print_data){
        print();
    }
}

void HfmEngine::save(void)
{
    if(hfm_engine_mass_air_flow) {
        hfm_engine_mass_air_flow->save_to_eeprom();
    }
    if(hfm_engine_max_mass_air_flow) {
        hfm_engine_max_mass_air_flow->save_to_eeprom();
    }
    if(print_data){
        print();
    }
}

void HfmEngine::print(void)
{
    const char* log_tag = "HFM_ENGINE";
    if(hfm_engine_mass_air_flow) {
        ESP_LOGI(log_tag, "Mass air flow map:");
        ESP_LOGI(log_tag, "0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000, 7250, 7500, 7750, 8000 rpm");
        int16_t* data = hfm_engine_mass_air_flow->get_current_data();
        for (uint16_t i = 0; i < HFM_ENGINE_TABLE_SIZE; i++)
        {
            ESP_LOGI(log_tag, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, // %.2f°", data[i*33u], data[i*33u+1u], data[i*33u+2u], data[i*33u+3u], data[i*33u+4u], data[i*33u+5u], data[i*33u+6u], data[i*33u+7u], data[i*33u+8u], data[i*33u+9u], data[i*33u+10u], data[i*33u+11u], data[i*33u+12u], data[i*33u+13u], data[i*33u+14u], data[i*33u+15u], data[i*33u+16u], data[i*33u+17u], data[i*33u+18u], data[i*33u+19u], data[i*33u+20u], data[i*33u+21u], data[i*33u+22u], data[i*33u+23u], data[i*33u+24u], data[i*33u+25u], data[i*33u+26u], data[i*33u+27u], data[i*33u+28u], data[i*33u+29u], data[i*33u+30u], data[i*33u+31u], data[i*33u+32u], (float)i * 0.35F);
        }
    }
    if(hfm_engine_max_mass_air_flow) {
        ESP_LOGI(log_tag, "Max Mass Air Flow Table:");
        ESP_LOGI(log_tag, "0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000, 7250, 7500, 7750, 8000 rpm");
        int16_t* data = hfm_engine_max_mass_air_flow->get_current_data();
        ESP_LOGI(log_tag, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31], data[32]);
    }
    if(hfm_engine_max_torque) {
        ESP_LOGI(log_tag, "Max Torque Table:");
        ESP_LOGI(log_tag, "0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000, 7250, 7500, 7750, 8000 rpm");
        int16_t* data = hfm_engine_max_torque->get_current_data();
        ESP_LOGI(log_tag, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31], data[32]);        
    }
}

HfmEngine *hfm_engine;