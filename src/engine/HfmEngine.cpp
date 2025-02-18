#include "HfmEngine.hpp"

HfmEngine::HfmEngine(/* args */)
{
}

HfmEngine::~HfmEngine()
{
}

bool HfmEngine::get_KLIMA(void)
{
    return false;
}

bool HfmEngine::get_AKT_R(void)
{
    return false;
}

bool HfmEngine::get_ANF2_EMS(void)
{
    return false;
}

bool HfmEngine::get_GHALT_EMS(void)
{
    return false;
}

bool HfmEngine::get_NOTL(void)
{
    return false;
}

bool HfmEngine::get_TM_REG(void)
{
    return false;
}

bool HfmEngine::get_GSQ(void)
{
    return false;
}

bool HfmEngine::get_WLAK(void)
{
    return false;
}

bool HfmEngine::get_N_MAX_BG(void)
{
    return false;
}

bool HfmEngine::get_KUEB_O_A(void)
{
    return false;
}

uint16_t HfmEngine::get_NMOT(void)
{
    return 0;
}

uint8_t HfmEngine::get_SLV(void)
{
    return 0;
}

float HfmEngine::get_ML(void)
{
    /* mass volumetric efficiency */
    float lambda_l = 0.0f;
    float m_Air = 0.0f;
    float m_theoretical = 0.0f;
    /* engine displacement [m³]*/
    float engine_displacement = ((float)V_H) * 0.001f;
    /* air density [kg/m³] */
    float rho_theoretical = 1.188f;
    /* temperature [K] */
    float T_theoretical = this->current_intake_air_temp; // T_LUFT    
    /* pressure [N/m²]*/
    float p_theoretical = this->current_air_pressure; // P_HOH
    rho_theoretical = T_theoretical / p_theoretical;

    m_theoretical = rho_theoretical * engine_displacement / ((float)n_cylinders);
    if (0.0f != m_theoretical) {
        lambda_l = m_Air / m_theoretical;
    }
    return lambda_l;
}

float HfmEngine::get_FMMOTMAX(void)
{
    return 0.0f;
}

uint16_t HfmEngine::get_MMOTI(void)
{
    return 0;
}

uint16_t HfmEngine::get_MMOTSCH(void)
{
    return 0;
}
