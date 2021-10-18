#include "scn.h"
#include "solenoids/solenoids.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp32/ulp.h>
#include "speaker.h"
#include "sensors.h"
#include "canbus/can_egs52.h"

bool setup_tcm()
{
    egs_can_hal = new Egs52Can("EGS52", 20);
    if (!egs_can_hal->begin_tasks()) {
        return false;
    }
    if (!Sensors::init_sensors()) {
        return false;
    }
    return init_all_solenoids();
}

void test_profiles(void*) {
    GearboxProfile profiles[7] = { 
        GearboxProfile::Agility, 
        GearboxProfile::Comfort, 
        GearboxProfile::Failure,
        GearboxProfile::Manual,
        GearboxProfile::Standard,
        GearboxProfile::Winter,
        GearboxProfile::Underscore
    };
    while(1) {
        for (int i = 0; i < 7; i++) {
            egs_can_hal->set_drive_profile(profiles[i]);
            vTaskDelay(1000);  
        }
    }
}

void printer(void*) {
    spkr.send_note(500, 200, 250);
    spkr.send_note(1000, 200, 200);
    int atf_temp;
    int vbatt;
    bool parking;
    uint32_t n2;
    uint32_t n3;
    while(1) {
        uint64_t start = esp_timer_get_time();
        Sensors::read_atf_temp(&atf_temp);
        Sensors::read_vbatt(&vbatt);
        Sensors::parking_lock_engaged(&parking);
        n2 = Sensors::read_n2_rpm();
        n3 = Sensors::read_n3_rpm();
        uint32_t taken = (uint32_t)(esp_timer_get_time() - start);
        ESP_LOGI(
            "MAIN", 
            "Y3: %d mA, Y4: %d mA, Y5: %d mA, MPC: %d mA, SPC: %d mA, TCC: %d mA. Vbatt: %d mV, ATF: %d *C, Parking lock: %d. N2/N3: (%u/%u) RPM. TIME: %u",
            sol_y3->get_current_estimate(),
            sol_y4->get_current_estimate(),
            sol_y5->get_current_estimate(),
            sol_mpc->get_current_estimate(),
            sol_spc->get_current_estimate(),
            sol_tcc->get_current_estimate(),
            vbatt,
            atf_temp,
            parking,
            n2, n3,
            taken
        );
        vTaskDelay(1000);
    }
}

extern "C" void app_main(void)
{
    if (setup_tcm() == false) { // An error ocurred setting up the gearbox!
        // Activate limp!
        egs_can_hal->set_drive_profile(GearboxProfile::Failure);
        egs_can_hal->set_display_msg(GearboxMessage::VisitWorkshop);
    }
    xTaskCreate(printer, "PRINTER", 8192, nullptr, 2, nullptr);
    xTaskCreate(test_profiles, "TEST-P", 4096, nullptr, 2, nullptr);
}