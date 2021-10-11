#include "scn.h"
#include "solenoids/solenoids.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp32/ulp.h>
#include "speaker.h"

void setup_tcm()
{
    init_all_solenoids();
    ESP_LOGI("INIT", "INIT OK!");
}

void set_note(uint32_t s, uint32_t duration, uint32_t total_duration) {
    spkr.set_freq(s);
    vTaskDelay(duration);
    spkr.set_freq(0);
    vTaskDelay(total_duration - duration);
}

void printer(void*) {
    set_note(500, 250, 300);
    set_note(500, 250, 300);
    while(1) {
        //ESP_LOGI("MAIN","RTC_SLOW_MEM[0] = %d", RTC_SLOW_MEM[1]);
        ESP_LOGI(
            "MAIN", 
            "Solenoid readings: Y3: %d mA, Y4: %d mA, Y5: %d mA, MPC: %d mA, SPC: %d mA, TCC: %d mA",
            sol_y3->get_current_estimate(),
            sol_y4->get_current_estimate(),
            sol_y5->get_current_estimate(),
            sol_mpc->get_current_estimate(),
            sol_spc->get_current_estimate(),
            sol_tcc->get_current_estimate()
        );
        vTaskDelay(1000);
    }
}

extern "C" void app_main(void)
{
    setup_tcm();
    xTaskCreate(printer, "PRINTER", 8192, nullptr, 2, nullptr);
}