#include "clock.hpp"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"

uint32_t TIMESTAMP_NOW = 0;

static bool IRAM_ATTR on_clock_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    TIMESTAMP_NOW = esp_timer_get_time()/1000;
    return true;
}

uint32_t GET_CLOCK_TIME() {
    return TIMESTAMP_NOW;
}

void init_clock() {
    // Start a timer that runs every 1ms
    gptimer_handle_t timer;
    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = (1 * 1000 * 1000),
        .flags = {
            .intr_shared = 1
        }
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));
    const gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        }
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_clock_timer_isr
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, nullptr));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}
