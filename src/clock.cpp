#include "clock.hpp"
#include "esp_timer.h"
#include "esp_attr.h"

uint32_t IRAM_ATTR GET_CLOCK_TIME() {
    return (esp_timer_get_time() / 1000);
}
