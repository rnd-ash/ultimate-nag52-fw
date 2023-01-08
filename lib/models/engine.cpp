#include "engine.h"
#include <math.h>
#include "../hal/hardwareabstractionlayer.h"

Engine::Engine(const float c, const LookupTable *_M_eng_max, const float throttle_diameter)
{
    r = throttle_diameter * DIAMETER_TO_RADIUS_FACTOR;
    A_throttle_valve = r * r * M_PI;
    c_eng = c;
    M_eng_max = _M_eng_max;
}

Engine::~Engine()
{}

uint16_t Engine::calculateTorque(const uint16_t engine_speed, const uint16_t maf)
{
    // engine speed needs to be converted from rpm to rph
    float engine_speed_float = ((float)engine_speed) * 60.F;
    float result = (c_eng * ((float)maf)) / engine_speed_float;
    return (uint16_t)result;
}

uint16_t Engine::estimateDemandedTorque(const uint16_t engine_speed, const float throttle_valve_opening_angle, const uint16_t p_a, const uint16_t T_a)
{
    // mdot = A_eff * p_u * sqrt(2/(R * T_a)) * psi(p_u, p_s)
    float b = cosf((throttle_valve_opening_angle * M_PI) / 180.F) * r;
    float A_eff = A_throttle_valve - (b * r * M_PI);
    uint16_t mdot = (uint16_t)(A_eff * p_a * sqrtf(2.F/(R * (float(T_a))) * psi(p_a, p_s)));
    return calculateTorque(engine_speed, mdot);
}

float Engine::psi(const uint16_t p_a, const uint16_t p_s)
{
    return 0.5F;
}

Engine::Torques Engine::getTorques(const uint16_t engine_speed, const float throttle_valve_opening_angle, const uint16_t T_a, const uint16_t p_a)
{
    return new Torques{
        .minTorque = 0u,
        .maxTorque = (uint16_t)(M_eng_max->getValue((float)engine_speed)),
        .demandedTorque = estimateDemandedTorque(engine_speed, throttle_valve_opening_angle),
        .staticTorque = calculateTorque(engine_speed, maf)        
    };
}
