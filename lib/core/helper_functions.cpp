#include "esp_log.h"
#include "helper_functions.h"

const char LOG_TAG[5] = "MISC";

bool check_esp_func(esp_err_t esp_err, const char* msg1) {
    bool result = (ESP_OK == esp_err);
    if(!result) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, LOG_TAG, "%s %s", msg1, esp_err_to_name(esp_err));    
    }
    return result;
}
