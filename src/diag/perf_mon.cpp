#include "perf_mon.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gptimer.h"

uint32_t _idle_ticks_c1 = 0;
uint32_t _idle_ticks_c2 = 0;

const uint32_t LOAD_FETCH_INTERVAL_MS = 100u;
const uint32_t TICKS_PER_SEC = 454967u; // IDF 5.0 barebones
const uint32_t MAX_TICKS = TICKS_PER_SEC/(1000u/LOAD_FETCH_INTERVAL_MS);
gptimer_handle_t gptimer = NULL;

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

// Guaranteed to run every 100ms
static bool IRAM_ATTR cpu_load_check(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    if (_idle_ticks_c1 <= MAX_TICKS)
    {
        dest.load_core_1 = 1000 - ((1000 * _idle_ticks_c1) / MAX_TICKS);
    }
    _idle_ticks_c1 = 0;
    if (_idle_ticks_c2 <= MAX_TICKS)
    {
        dest.load_core_2 = 1000 - ((1000 * _idle_ticks_c2) / MAX_TICKS);
    }
    _idle_ticks_c2 = 0;
    return true;
}

esp_err_t init_perfmon(void)
{
    if (!perfmon_running)
    {
        dest.load_core_1 = 0;
        dest.load_core_2 = 0;
        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1 * 1000 * 1000, // 1 tick = 1ms
            .flags = {
                .intr_shared = 0
            }
        };
        ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, &gptimer), "PERFMON", "Failed to create new GPTIMER");

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = (LOAD_FETCH_INTERVAL_MS * 1000),
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = true,
            }
        };
        ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(gptimer, &alarm_config), "PERFMON", "Failed to set GPTIMER Alarm action");
        gptimer_event_callbacks_t cbs = {
            .on_alarm = cpu_load_check
        };
        ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(gptimer, &cbs, nullptr), "PERFMON", "Failed to register GPTIMER callback");
        ESP_RETURN_ON_ERROR(gptimer_enable(gptimer), "PERFMON", "Failed to enable GPTIMER");
        ESP_RETURN_ON_ERROR(gptimer_start(gptimer), "PERFMON", "Failed to start GPTIMER");
        ESP_RETURN_ON_ERROR(esp_register_freertos_idle_hook_for_cpu(idle_hook_c1, 0), "PERFMON", "Failed to set idle hook for Core 0");
        ESP_RETURN_ON_ERROR(esp_register_freertos_idle_hook_for_cpu(idle_hook_c2, 1), "PERFMON", "Failed to set idle hook for Core 1");
        perfmon_running = true;
        ESP_LOG_LEVEL(ESP_LOG_INFO, "PERFMON", "Init OK!");
    }
    return perfmon_running;
}

void remove_perfmon(void)
{
    gptimer_disable(gptimer);
    perfmon_running = false;
}

CpuStats get_cpu_stats(void)
{
    return dest;
}