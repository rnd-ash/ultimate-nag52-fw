#include "esp_log.h"

#define LOG_TAG "MISC"

#define CHECK_ESP_FUNC(x, msg, ...) \
    res = x; \
    if (res != ESP_OK) { \
        ESP_LOGE(LOG_TAG, msg, ##__VA_ARGS__); \
        return false; \
    }   \
