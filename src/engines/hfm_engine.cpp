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
    const int16_t* default_data_maf = HFM_ENGINE_MASS_AIR_FLOW;
    hfm_engine_mass_air_flow = new StoredMap(key_name_maf, HFM_ENGINE_MAP_SIZE, ENGINE_RPM_HEADERS, engine_throttle_headers, HFM_ENGINE_TABLE_SIZE, HFM_ENGINE_TABLE_SIZE, default_data_maf);
    
    const char* key_name_max_maf = NVS_KEY_MAP_NAME_HFM_ENGINE_MAX_MASS_AIR_FLOW;
    const int16_t* default_data_max_maf = HFM_ENGINE_MAX_MASS_AIR_FLOW;
    hfm_engine_max_mass_air_flow = new StoredTable(key_name_max_maf, HFM_ENGINE_TABLE_SIZE, ENGINE_RPM_HEADERS, HFM_ENGINE_TABLE_SIZE, default_data_max_maf);    
}

HfmEngine::~HfmEngine()
{
}

int16_t HfmEngine::get_max_torque(uint16_t n_mot)
{
    return (int16_t)(hfm_engine_max_torque->get_value(n_mot));
}

float HfmEngine::get_mass_air_flow(uint16_t n_mot, uint8_t throttle)
{
    return 0;
}

float HfmEngine::get_max_mass_air_flow(uint16_t n_mot)
{
    return hfm_engine_max_mass_air_flow->get_value(n_mot);
}

bool HfmEngine::add_to_mass_air_flow_map(uint16_t n_mot, uint8_t throttle, float mass_air_flow)
{
    if(throttle == VEHICLE_CONFIG.throttlevalve_maxopeningangle){
        // at full throttle, indicated torque is equal to maximum torque
        hfm_engine_max_mass_air_flow->add_value(n_mot, (int16_t)mass_air_flow, 0.0F);
    }
    return hfm_engine_mass_air_flow->add_value(n_mot, throttle, (int16_t)mass_air_flow, 0.0F);
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

    m_theoretical = rho_theoretical * V_H / ((float)n_cylinders);
    if (0.0f != m_theoretical)
    {
        lambda_l = m_Air / m_theoretical;
    }
    return lambda_l;
}

void HfmEngine::save(void)
{
    if(hfm_engine_mass_air_flow) {
        hfm_engine_mass_air_flow->save_to_eeprom();
    }
    if(hfm_engine_max_mass_air_flow) {
        hfm_engine_max_mass_air_flow->save_to_eeprom();
    }
}

HfmEngine *hfm_engine;