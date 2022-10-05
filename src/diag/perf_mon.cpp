#include "perf_mon.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <esp_timer.h>
#include <driver/timer.h>
#include "macros.h"

#define LOAD_FETCH_INTERVAL_MS 100

#ifdef LOG_TAG
    #undef LOG_TAG
#endif
#define LOG_TAG "PERFMON"



// 346834 per second

volatile uint64_t _idle_ticks_c1 = 0;
volatile uint64_t _idle_ticks_c2 = 0;

#define TICKS_PER_MS 371000.0 / 1000.0

const uint64_t MAX_TICKS = TICKS_PER_MS * LOAD_FETCH_INTERVAL_MS;

CpuStats dest;
bool perfmon_running = false;

static bool idle_hook_c1() {
    _idle_ticks_c1++;
    return false;
}

static bool idle_hook_c2() {
    _idle_ticks_c2++;
    return false;
}

uint64_t i0 = 0;
uint64_t i1 = 0;

// Guaranteed to run every 100ms
static intr_handle_t load_timer_handle;
static void IRAM_ATTR cpu_load_interrupt(void* arg) {
    TIMERG0.int_clr_timers.t1 = 1;
    TIMERG0.hw_timer[1].config.alarm_en = 1;
    i0 = _idle_ticks_c1;
    i1 = _idle_ticks_c2;
    _idle_ticks_c1 = 0;
    _idle_ticks_c2 = 0;
    if (i0 <= MAX_TICKS) {
        dest.load_core_1 = 1000 - ((1000 * i0)/MAX_TICKS);
    }
    if (i1 <= MAX_TICKS) {
        dest.load_core_2 = 1000 - ((1000 * i1)/MAX_TICKS);
    }
}

bool init_perfmon() {
    if (perfmon_running) {
        return true;
    }

    timer_config_t config = {
            .alarm_en = timer_alarm_t::TIMER_ALARM_EN,
            .counter_en = timer_start_t::TIMER_START,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = timer_autoreload_t::TIMER_AUTORELOAD_EN,
            .divider = 80   /* 1 us per tick */
    };

    dest.load_core_1 = 0;
    dest.load_core_2 = 0;

    esp_err_t res;
    CHECK_ESP_FUNC(timer_init(TIMER_GROUP_0, TIMER_1, &config), "Timer init failed: %s", esp_err_to_name(res));
    CHECK_ESP_FUNC(timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0), "Set counter value failed: %s", esp_err_to_name(res));
    CHECK_ESP_FUNC(timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, LOAD_FETCH_INTERVAL_MS*1000), "Set alarm value failed: %s", esp_err_to_name(res));
    CHECK_ESP_FUNC(timer_enable_intr(TIMER_GROUP_0, TIMER_1), "Enable intr failed: %s", esp_err_to_name(res));
    CHECK_ESP_FUNC(timer_isr_register(TIMER_GROUP_0, TIMER_1, &cpu_load_interrupt, NULL, 0, &load_timer_handle), "ISR Register failed: %s", esp_err_to_name(res));
    CHECK_ESP_FUNC(timer_start(TIMER_GROUP_0, TIMER_1), "Timer start failed: %s", esp_err_to_name(res));
    CHECK_ESP_FUNC(esp_register_freertos_idle_hook_for_cpu(idle_hook_c1, 0), "Failed to set idle hook for Core 0: %s", esp_err_to_name(res));
    CHECK_ESP_FUNC(esp_register_freertos_idle_hook_for_cpu(idle_hook_c2, 1), "Failed to set idle hook for Core 1: %s", esp_err_to_name(res));
    perfmon_running = true;
    ESP_LOG_LEVEL(ESP_LOG_INFO, LOG_TAG, "Init OK!");
    return true;
}

void remove_perfmon() {
    timer_deinit(TIMER_GROUP_0, TIMER_1);
    perfmon_running = false;
}

CpuStats get_cpu_stats() {
    return dest;
}