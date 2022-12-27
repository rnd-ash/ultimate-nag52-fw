#include "engine.h"
#include <math.h>
#include "esp_heap_caps.h"
#include

Engine::Engine(float c, uint16_t *M_eng_max, uint16_t *M_eng_max_headers, uint16_t length_M_eng_max, float throttle_diameter)
{
    r = throttle_diameter * DIAMETER_TO_RADIUS_FACTOR;
    A_throttle_valve = r * r * M_PI;
    c_eng = c;
    length_M_eng_max_loc = length_M_eng_max;

    M_eng_max_loc = static_cast<uint16_t>(heap_caps_malloc(length_M_eng_max_loc * sizeof(uint16_t), MALLOC_CAP_SPIRAM));
    M_eng_max_headers_loc = static_cast<uint16_t>(heap_caps_malloc(length_M_eng_max_loc * sizeof(uint16_t), MALLOC_CAP_SPIRAM));
    
    allocation_succeeded = (nullptr != M_eng_max_loc) && (nullptr != M_eng_max_headers_loc);

    if(allocation_succeeded){
        (void)memcpy(M_eng_max_loc, M_eng_max, sizeof(uint16_t) * length_M_eng_max_loc);
        (void)memcpy(M_eng_max_headers_loc, M_eng_headers_max, sizeof(uint16_t) * length_M_eng_max_loc);
    }
}

Engine::~Engine()
{
    if(allocation_succeeded){
        // free the memory
        heap_caps_free(M_eng_max_loc);
        heap_caps_free(M_eng_max_headers_loc);
    }
}

Engine::Torques Engine::getTorques(uint16_t engine_speed, float throttle_valve_flare_angle, uint16_t p_a)
{
    return Torques();
}
