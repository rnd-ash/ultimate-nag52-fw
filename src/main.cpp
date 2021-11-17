#include "scn.h"
#include "solenoids/solenoids.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp32/ulp.h>
#include "speaker.h"
#include "sensors.h"
#include "canbus/can_egs52.h"
//#include "canbus/can_egs53.h"
#include "gearbox.h"
#include "dtcs.h"
#include "nvs/eeprom_config.h"

#define NUM_PROFILES 5 // A, C, W, M, S

Gearbox* gearbox;

AgilityProfile* agility;
ComfortProfile* comfort;
WinterProfile* winter;
ManualProfile* manual;
StandardProfile* standard;

uint8_t profile_id = 0;
AbstractProfile* profiles[NUM_PROFILES];

SPEAKER_POST_CODE setup_tcm()
{
#ifdef EGS52_MODE
    egs_can_hal = new Egs52Can("EGS52", 20); // EGS52 CAN Abstraction layer
#endif
#ifdef EGS53_MODE
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

    agility = new AgilityProfile();
    comfort = new ComfortProfile();
    winter = new WinterProfile();
    manual = new ManualProfile();
    standard = new StandardProfile();

    profiles[0] = manual;
    profiles[1] = comfort;
    profiles[2] = agility;
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
            vTaskDelay(2000);
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
    //spkr.broadcast_error_code(DtcCode::P2005);
    //spkr.broadcast_error_code(DtcCode::P2564);
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
        vTaskDelay(1000);
        //SOL->write_pwm_percent(pwm);
        //vTaskDelay(500);
        //ESP_LOGI("SOL", "PWM %d EST: %d mA", pwm, SOL->get_current_estimate());
        //vTaskDelay(1000);
        //SOL->write_pwm_percent(0);
        //vTaskDelay(1000);
        //pwm += 250;
        //if (pwm > 1000) {
        //    pwm = 0;
        //}
    }
}

void input_manager(void*) {
    bool pressed = false;
    PaddlePosition last_pos = PaddlePosition::None;
    ShifterPosition slast_pos = ShifterPosition::SignalNotAvaliable;
    while(1) {
        uint64_t now = esp_timer_get_time();
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
        vTaskDelay(20);
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
            vTaskDelay(1000);
        }
    } else { // INIT OK!
        xTaskCreate(input_manager, "INPUT_MANAGER", 8192, nullptr, 5, nullptr);
        //xTaskCreate(printer, "PRINTER", 4096, nullptr, 2, nullptr);
    }
}