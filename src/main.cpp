#include "scn.h"
#include "solenoids/solenoids.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp32/ulp.h>
#include "speaker.h"
#include "sensors.h"
#include "canbus/can_egs52.h"
#include "canbus/can_egs53.h"
#include <gearbox_config.h>
#include "gearbox.h"
#include "dtcs.h"
#include "nvs/eeprom_config.h"
#include "diag/kwp2000.h"

#define NUM_PROFILES 5 // A, C, W, M, S

// Sanity check
#if !defined(EGS52_MODE) && !defined(EGS53_MODE)
    #error "No CAN definition (EGS52/EGS53)"
#endif

#if defined(EGS52_MODE) && defined(EGS53_MODE)
    #error "Both EGS52 and EGS53 modes CANNOT be enabled at the same time!"
#endif

Gearbox* gearbox;
Kwp2000_server* diag_server;

uint8_t profile_id = 0;
AbstractProfile* profiles[NUM_PROFILES];

SPEAKER_POST_CODE setup_tcm()
{
#ifdef EGS52_MODE
    #warning "Building with EGS52 CAN support"
    egs_can_hal = new Egs52Can("EGS52", 20); // EGS52 CAN Abstraction layer
#endif
#ifdef EGS53_MODE
    #warning "Building with EGS53 CAN support"
    egs_can_hal = new Egs53Can("EGS53", 20); // EGS53 CAN Abstraction layer
#endif
    if (!egs_can_hal->begin_tasks()) {
        return SPEAKER_POST_CODE::CAN_FAIL;
    }
    if (!Sensors::init_sensors()) {
        return SPEAKER_POST_CODE::SENSOR_FAIL;
    }
    if(!init_all_solenoids()) {
        return SPEAKER_POST_CODE::SOLENOID_FAIL;
    }

    profiles[0] = manual;
    profiles[1] = agility;
    profiles[2] = comfort;
    profiles[3] = standard;
    profiles[4] = winter;

    if (!EEPROM::init_eeprom()) {
        return SPEAKER_POST_CODE::EEPROM_FAIL;
    }

    gearbox = new Gearbox();
    if (!gearbox->start_controller()) {
        return SPEAKER_POST_CODE::CONTROLLER_FAIL;
    }
    gearbox->set_profile(profiles[0]);
    return SPEAKER_POST_CODE::INIT_OK;
}

void err_beep_loop(void* a) {
    SPEAKER_POST_CODE p = (SPEAKER_POST_CODE)(int)a;
    if (p == SPEAKER_POST_CODE::INIT_OK) {
        spkr.post(p); // All good, return
        vTaskDelete(NULL);
    } else {
        // An error has occurred
        // Set gearbox to F mode
        egs_can_hal->set_drive_profile(GearboxProfile::Failure);
        egs_can_hal->set_display_msg(GearboxMessage::VisitWorkshop);
        egs_can_hal->set_gearbox_ok(false);
        while(1) {
            spkr.post(p);
            vTaskDelay(2000/portTICK_RATE_MS);
        }
        vTaskDelete(NULL);
    }
}


void printer(void*) {
    int atf_temp;
    uint16_t vbatt;
    bool parking;
    uint32_t n2;
    uint32_t n3;
    int x = 0;
    while(1) {
        Sensors::read_atf_temp(&atf_temp);
        Sensors::read_vbatt(&vbatt);
        Sensors::parking_lock_engaged(&parking);
        n2 = Sensors::read_n2_rpm();
        n3 = Sensors::read_n3_rpm();
        ESP_LOGI(
            "MAIN", 
            "Y3: %d mA, Y4: %d mA, Y5: %d mA, MPC: %d mA, SPC: %d mA, TCC: %d mA. N2/N3: (%u/%u) RPM.",
            sol_y3->get_current_estimate(),
            sol_y4->get_current_estimate(),
            sol_y5->get_current_estimate(),
            sol_mpc->get_current_estimate(),
            sol_spc->get_current_estimate(),
            sol_tcc->get_current_estimate(),
            n2, n3
        );
        vTaskDelay(2000/portTICK_RATE_MS);
    }
}

void input_manager(void*) {
    bool pressed = false;
    PaddlePosition last_pos = PaddlePosition::None;
    ShifterPosition slast_pos = ShifterPosition::SignalNotAvaliable;
    while(1) {
        uint64_t now = esp_timer_get_time()/1000;
        bool down = egs_can_hal->get_profile_btn_press(now, 100);
        if (down) {
            pressed = true;
        } else { // Released
            if (pressed) {
                pressed = false; // Released, do thing now
                if (egs_can_hal->get_shifter_position_ewm(now, 100) == ShifterPosition::PLUS) {
                    gearbox->inc_subprofile();
                } else {
                    profile_id++;
                    if (profile_id == NUM_PROFILES) {
                        profile_id = 0;
                    }
                    gearbox->set_profile(profiles[profile_id]);  
                }
            }
        }
        PaddlePosition paddle = egs_can_hal->get_paddle_position(now, 100);
        if (last_pos != paddle) { // Same position, ignore
            if (last_pos != PaddlePosition::None) {
                // Process last request of the user
                if (last_pos == PaddlePosition::Plus) {
                    gearbox->inc_gear_request();
                } else if (last_pos == PaddlePosition::Minus) {
                    gearbox->dec_gear_request();
                }
            }
            last_pos = paddle;
        }
        ShifterPosition spos = egs_can_hal->get_shifter_position_ewm(now, 100);
        if (spos != slast_pos) { // Same position, ignore
            // Process last request of the user
            if (slast_pos == ShifterPosition::PLUS) {
                gearbox->inc_gear_request();
            } else if (slast_pos == ShifterPosition::MINUS) {
                gearbox->dec_gear_request();
            }
            slast_pos = spos;
        }
        vTaskDelay(20/portTICK_RATE_MS);
    }
}

const char* post_code_to_str(SPEAKER_POST_CODE s) {
    switch (s) {
        case SPEAKER_POST_CODE::INIT_OK:
            return "INIT_OK";
        case SPEAKER_POST_CODE::CAN_FAIL:
            return "CAN_INIT_FAIL";
        case SPEAKER_POST_CODE::CONTROLLER_FAIL:
            return "CONTROLLER_INIT_FAIL";
        case SPEAKER_POST_CODE::EEPROM_FAIL:
            return "ERRPOM_INIT_FAIL";
        case SPEAKER_POST_CODE::SENSOR_FAIL:
            return "SENSOR_INIT_FAIL";
        case SPEAKER_POST_CODE::SOLENOID_FAIL:
            return "SOLENOID_INIT_FAIL";
        default:
            return nullptr;
    }
}

extern "C" void app_main(void)
{
    SPEAKER_POST_CODE s = setup_tcm();
    xTaskCreate(err_beep_loop, "PCSPKR", 2048, (void*)s, 2, nullptr);
    if (s != SPEAKER_POST_CODE::INIT_OK) {
        while(true) {
            ESP_LOGE("INIT", "TCM INIT ERROR (%s)! CANNOT START TCM!", post_code_to_str(s));
            vTaskDelay(1000/portTICK_RATE_MS);
        }
    } else { // INIT OK!
        xTaskCreate(input_manager, "INPUT_MANAGER", 8192, nullptr, 5, nullptr);
        xTaskCreate(printer, "PRINTER", 4096, nullptr, 2, nullptr);
    }
    // Now spin up the KWP2000 server (last thing)
    diag_server = new Kwp2000_server(egs_can_hal, gearbox);
    xTaskCreatePinnedToCore(Kwp2000_server::start_kwp_server, "KWP2000", 32*1024, diag_server, 5, nullptr, 0);
    //xTaskCreate(solenoid_test, "TEST_LOOP", 8192, nullptr, 2, nullptr);
}