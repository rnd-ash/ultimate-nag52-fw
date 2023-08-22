#include "perf_mon.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include "clock.hpp"

uint32_t _idle_ticks_c1 = 0;
uint32_t _idle_ticks_c2 = 0;
const uint32_t TICKS_PER_SEC = 352989; // IDF 5.0 barebones
uint32_t last_update_time = 0;
CpuStats dest;

static bool idle_hook_c1(void)
{
    _idle_ticks_c1++;
    return false;
}

static bool idle_hook_c2(void)
{
    _idle_ticks_c2++;
    return false;
}

void PerfMon::init_perfmon(void) {
    esp_register_freertos_idle_hook_for_cpu(idle_hook_c1, 0);
    esp_register_freertos_idle_hook_for_cpu(idle_hook_c2, 1);
}

void PerfMon::update_sample(void) {

    float ms_elapsed = GET_CLOCK_TIME() - last_update_time;
    float max_ticks = (float)TICKS_PER_SEC * (ms_elapsed / 1000.0);
    if (_idle_ticks_c1 <= max_ticks)
    {
        float free = (float)_idle_ticks_c1/max_ticks;
        dest.load_core_1 = 1000.0 - (1000.0*free);
    }
    _idle_ticks_c1 = 0;
    if (_idle_ticks_c2 <= max_ticks)
    {
        float free = (float)_idle_ticks_c2/max_ticks;
        dest.load_core_2 = 1000.0 - (1000.0*free);
    }
    _idle_ticks_c2 = 0;
    last_update_time = GET_CLOCK_TIME();

   /*
   // SEE MAIN.c. Uncomment this code when calibrating tick count
   printf("%lu %lu\n", _idle_ticks_c1, _idle_ticks_c2);
   _idle_ticks_c1 = 0;
   _idle_ticks_c2 = 0;
   */
}

CpuStats PerfMon::get_cpu_stats(void)
{
    return dest;
}