#include "esp_log.h"

#define LOG_TAG "MISC"

#define CHECK_ESP_FUNC(x, msg, ...) \
    res = x; \
    if (res != ESP_OK) { \
        ESP_LOG_LEVEL(ESP_LOG_ERROR, LOG_TAG, msg, ##__VA_ARGS__); \
        return false; \
    }   \


#define CLAMP(value, min, max) \
    if (value < min) { \
        value = min; \
    } else if (value >= max) { \
        value = max-1; \
    }