#include "perf_mon.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <esp_timer.h>
#include <driver/timer.h>
#include "esp_log.h"
#include "esp_check.h"

// 346834 per second

volatile uint64_t _idle_ticks_c1 = 0;
volatile uint64_t _idle_ticks_c2 = 0;

const uint64_t LOAD_FETCH_INTERVAL_MS = 100u;
const uint64_t TICKS_PER_MS = 371000u / 1000u; 

const uint64_t MAX_TICKS = TICKS_PER_MS * LOAD_FETCH_INTERVAL_MS;

CpuStats dest;
bool perfmon_running = false;

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

uint64_t i0 = 0;
uint64_t i1 = 0;

// Guaranteed to run every 100ms
static intr_handle_t load_timer_handle;
static void IRAM_ATTR cpu_load_interrupt(void *arg)
{
    TIMERG0.int_clr_timers.t1 = 1;
    TIMERG0.hw_timer[1].config.alarm_en = 1;
    i0 = _idle_ticks_c1;
    i1 = _idle_ticks_c2;
    _idle_ticks_c1 = 0;
    _idle_ticks_c2 = 0;
    if (i0 <= MAX_TICKS)
    {
        dest.load_core_1 = 1000 - ((1000 * i0) / MAX_TICKS);
    }
    if (i1 <= MAX_TICKS)
    {
        dest.load_core_2 = 1000 - ((1000 * i1) / MAX_TICKS);
    }
}

const static timer_config_t PERFORM_T_CONFIG = {
    .alarm_en = timer_alarm_t::TIMER_ALARM_EN,
    .counter_en = timer_start_t::TIMER_START,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = timer_autoreload_t::TIMER_AUTORELOAD_EN,
    .divider = 80 /* 1 us per tick */
};

esp_err_t init_perfmon(void)
{
    if (!perfmon_running)
    {
        dest.load_core_1 = 0;
        dest.load_core_2 = 0;

        ESP_RETURN_ON_ERROR(timer_init(TIMER_GROUP_0, TIMER_1, &PERFORM_T_CONFIG), "PERFMON", "Timer init failed");
        ESP_RETURN_ON_ERROR(timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0), "PERFMON", "Set counter value failed");
        ESP_RETURN_ON_ERROR(timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, LOAD_FETCH_INTERVAL_MS * 1000), "PERFMON", "Set alarm value failed");
        ESP_RETURN_ON_ERROR(timer_enable_intr(TIMER_GROUP_0, TIMER_1), "PERFMON", "Enable intr failed");
        ESP_RETURN_ON_ERROR(timer_isr_register(TIMER_GROUP_0, TIMER_1, &cpu_load_interrupt, NULL, 0, &load_timer_handle), "PERFMON", "ISR Register failed");
        ESP_RETURN_ON_ERROR(timer_start(TIMER_GROUP_0, TIMER_1), "PERFMON", "Timer start failed");
        ESP_RETURN_ON_ERROR(esp_register_freertos_idle_hook_for_cpu(idle_hook_c1, 0), "PERFMON", "Failed to set idle hook for Core 0");
        ESP_RETURN_ON_ERROR(esp_register_freertos_idle_hook_for_cpu(idle_hook_c2, 1), "PERFMON", "Failed to set idle hook for Core 1");
        perfmon_running = true;
        ESP_LOG_LEVEL(ESP_LOG_INFO, "PERFMON", "Init OK!");
    }
    return perfmon_running;
}

void remove_perfmon(void)
{
    timer_deinit(TIMER_GROUP_0, TIMER_1);
    perfmon_running = false;
}

CpuStats get_cpu_stats(void)
{
    return dest;
}