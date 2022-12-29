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
{
    if(allocation_succeeded){
        // free the memory
        heap_caps_free(M_eng_max_loc);
        heap_caps_free(M_eng_max_headers_loc);
    }
}

uint16_t Engine::getStaticTorque(const uint16_t engine_speed, const uint16_t maf)
{
    // engine speed needs to be converted from rpm to rph
    float engine_speed_float = ((float)engine_speed) * 60.F;
    float result =(c_eng * ((float)maf)) / engine_speed_float;
    return (uint16_t)result;
}

Engine::Torques Engine::getTorques(const uint16_t engine_speed, const float throttle_valve_flare_angle, const uint16_t p_a)
{
    return new Torques{
        .minTorque = 0u,
        .maxTorque = (uint16_t)(M_eng_max->getValue((float)engine_speed)),
        .demandedTorque = 0u,
        .staticTorque = getStaticTorque(engine_speed, maf)        
    };
}
