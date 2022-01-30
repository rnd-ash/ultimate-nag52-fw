#include "perf_mon.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <esp_timer.h>

volatile uint64_t _idle_ticks_c1 = 0;
volatile uint64_t _idle_ticks_c2 = 0;
volatile uint64_t last_measure_time = 0;

#define TICKS_PER_MS 371

static bool idle_hook_c1() {
    _idle_ticks_c1++;
    return false;
}

static bool idle_hook_c2() {
    _idle_ticks_c2++;
    return false;
}

void init_perfmon() {
    esp_register_freertos_idle_hook_for_cpu(idle_hook_c1, 0);
    esp_register_freertos_idle_hook_for_cpu(idle_hook_c2, 1);
}

uint16_t c1;
uint16_t c2;
CpuStats last_report = {0,0};

CpuStats get_cpu_usage() {
    uint64_t now = esp_timer_get_time()/1000;
    if (now-last_measure_time < 500) {
        return last_report;
    };
    float i0 = _idle_ticks_c1;
    float i1 = _idle_ticks_c2;
    float max_ticks = TICKS_PER_MS*(now-last_measure_time);
    if (i0 > max_ticks) {
        c1 = 0;
    } else {
        c1 = 1000 - (1000 * (float)i0/max_ticks);
    }
    if (i1 > max_ticks) {
        c2 = 0;
    } else {
        c2 = 1000 - (1000 * (float)i1/max_ticks);
    }
    last_measure_time = now;
    _idle_ticks_c1 = 0;
    _idle_ticks_c2 = 0;
    last_report = CpuStats {
        .load_core_1 = c1,
        .load_core_2 = c2,
    };
    return last_report;
}