#include "scn.h"
#include "solenoids/solenoids.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp32/ulp.h>
#include "speaker.h"

#include "canbus/can_egs52.h"

void setup_tcm()
{
    egs_can_hal = new Egs52Can("EGS52", 20);
    if (!egs_can_hal->begin_tasks()) {
        return;
    }
    init_all_solenoids();
    ESP_LOGI("INIT", "INIT OK!");
}

void printer(void*) {
    spkr.send_note(500, 200, 250);
    spkr.send_note(1000, 200, 200);
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
        switch(egs_can_hal->get_engine_type()) {
            case EngineType::Diesel:
                ESP_LOGI("ENG_CHECK", "Engine is diesel");
            case EngineType::Petrol:
                ESP_LOGI("ENG_CHECK", "Engine is petrol");
            case EngineType::Unknown:
                ESP_LOGI("ENG_CHECK", "Engine is unknown");
            default:
                break;
        }
        vTaskDelay(1000);
    }
}

extern "C" void app_main(void)
{
    setup_tcm();
    xTaskCreate(printer, "PRINTER", 8192, nullptr, 2, nullptr);
}