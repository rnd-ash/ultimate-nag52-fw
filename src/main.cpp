#ifndef UNIT_TEST

#include "clock.hpp"

#include "solenoids/solenoids.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "speaker.h"
#include "sensors.h"
#include "gearbox.h"
#include "dtcs.h"
#include "nvs/eeprom_config.h"
#include "diag/kwp2000.h"
#include "nvs/module_settings.h"

// CAN LAYERS
#include "canbus/can_egs51.h"
#include "canbus/can_egs52.h"
#include "canbus/can_egs53.h"
#include "canbus/can_hfm.h"
#include "board_config.h"

Kwp2000_server* diag_server;

uint8_t profile_id = 0;
#define NUM_PROFILES 6 // A, C, W, M, S, R
AbstractProfile* profiles[NUM_PROFILES];

Speaker* spkr2 = nullptr;

SPEAKER_POST_CODE setup_tcm() {
    SPEAKER_POST_CODE ret = SPEAKER_POST_CODE::INIT_OK; // OK by default
    if (EEPROM::read_efuse_config(&BOARD_CONFIG) != ESP_OK) {
        ret = SPEAKER_POST_CODE::EFUSE_NOT_SET;
    } else {
        // First thing to do, Configure the GPIO Pin matrix!
        switch (BOARD_CONFIG.board_ver) {
            case 1:
                pcb_gpio_matrix = new BoardV11GpioMatrix();
                break;
            case 2:
                pcb_gpio_matrix = new BoardV12GpioMatrix();
                break;
            case 3:
                pcb_gpio_matrix = new BoardV13GpioMatrix();
                break;
            default:
                pcb_gpio_matrix = nullptr;
                egs_can_hal = nullptr;
                spkr = new Speaker(gpio_num_t::GPIO_NUM_4); // Assume legacy when this fails!
                spkr2 = new Speaker(gpio_num_t::GPIO_NUM_0); // For new PCBs
                ret = SPEAKER_POST_CODE::EFUSE_NOT_SET;
        }
        if (ret == SPEAKER_POST_CODE::INIT_OK) {
            spkr = new Speaker(pcb_gpio_matrix->spkr_pin);
            if (EEPROM::init_eeprom() != ESP_OK) {
                ret = SPEAKER_POST_CODE::EEPROM_FAIL;
            } else {
                // Read our configuration (This is allowed to fail as the default opts are always set by default)
                ModuleConfiguration::load_all_settings();
                switch (VEHICLE_CONFIG.egs_can_type) {
                    case 1:
                        egs_can_hal = new Egs51Can("EGS51", 20, 500000); // EGS51 CAN Abstraction layer
                        break;
                    case 2:
                        egs_can_hal = new Egs52Can("EGS52", 20, 500000); // EGS52 CAN Abstraction layer
                        break;
                    case 3:
                        egs_can_hal = new Egs53Can("EGS53", 20, 500000); // EGS53 CAN Abstraction layer
                        break;
                    case 4:
                        egs_can_hal = new HfmCan("HFM", 20, 125000); // HFM CAN Abstraction layer
                        break;
                    default:
                        // Unknown (Fallback to basic CAN)
                        ESP_LOGE("INIT", "ERROR. CAN Mode not set, falling back to basic CAN (Diag only!)");
                        egs_can_hal = new EgsBaseCan("EGSBASIC", 20, 500000);
                        break;
                }
            }
            if (!egs_can_hal->begin_tasks()) {
                ret = SPEAKER_POST_CODE::CAN_FAIL;
            } else {
                if (Sensors::init_sensors() != ESP_OK) {
                    ret = SPEAKER_POST_CODE::SENSOR_FAIL;
                } else {
                    if(Solenoids::init_all_solenoids() != ESP_OK) {
                        ret = SPEAKER_POST_CODE::SOLENOID_FAIL;
                    }
                }
            }
        }
    }
    if (ret == SPEAKER_POST_CODE::INIT_OK) {
        standard = new StandardProfile(VEHICLE_CONFIG.engine_type == 0);
        comfort = new ComfortProfile(VEHICLE_CONFIG.engine_type == 0);
        winter = new WinterProfile(VEHICLE_CONFIG.engine_type == 0);
        agility = new AgilityProfile(VEHICLE_CONFIG.engine_type == 0);
        manual = new ManualProfile(VEHICLE_CONFIG.engine_type == 0);
        race = new RaceProfile(VEHICLE_CONFIG.engine_type == 0);

        profiles[0] = standard;
        profiles[1] = comfort;
        profiles[2] = winter;
        profiles[3] = agility;
        profiles[4] = manual;
        profiles[5] = race;

        // Read profile ID on startup based on TCM config
        profile_id = VEHICLE_CONFIG.default_profile;
        if (profile_id > 4) {
            profile_id = 0;
        }

        gearbox = new Gearbox();
        if (gearbox->start_controller() != ESP_OK) {
            ret = SPEAKER_POST_CODE::CONTROLLER_FAIL;
        } else {
            gearbox->set_profile(profiles[profile_id]);
        }
    }
    return ret;
}

void err_beep_loop(void* a) {
    SPEAKER_POST_CODE p = (SPEAKER_POST_CODE)(int)a;
    if (p == SPEAKER_POST_CODE::INIT_OK) {
        spkr->post(p); // All good, return
        egs_can_hal->set_gearbox_ok(true);
        vTaskDelete(NULL);
    } else {
        if (egs_can_hal != nullptr) {
            // An error has occurred
            // Set gearbox to F mode
            egs_can_hal->set_drive_profile(GearboxProfile::Failure);
            egs_can_hal->set_display_msg(GearboxMessage::VisitWorkshop);
            egs_can_hal->set_gearbox_ok(false);
        }
        while(1) {
            spkr->post(p);
            if (spkr2 != nullptr) {
                spkr2->post(p);
            }
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
}

AbstractProfile* profile_from_auto_ty(AutoProfile prof) {
    AbstractProfile* p = standard;
    switch(prof) {
        case AutoProfile::Sport:
            p = standard;
            break;
        case AutoProfile::Agility:
            p = agility;
            break;
        case AutoProfile::Winter:
        case AutoProfile::Comfort:
        default:
            return comfort;
    }
    return p;
}

void input_manager(void*) {
    PaddlePosition last_pos = PaddlePosition::None;
    ShifterPosition slast_pos = ShifterPosition::SignalNotAvailable;

    uint32_t now = GET_CLOCK_TIME();
    ProfileSwitchPos last_mode = egs_can_hal->get_shifter_ws_mode(now, 100);
    bool pressed = false;
    
    if(ETS_CURRENT_SETTINGS.ewm_selector_type == EwmSelectorType::Switch || VEHICLE_CONFIG.shifter_style == 1) {
        gearbox->set_profile(profile_from_auto_ty(ETS_CURRENT_SETTINGS.profile_idx_buttom));
        if (last_mode != ProfileSwitchPos::SNV) {
            gearbox->set_profile(profile_from_auto_ty(last_mode == ProfileSwitchPos::Top ? ETS_CURRENT_SETTINGS.profile_idx_top : ETS_CURRENT_SETTINGS.profile_idx_buttom)); 
        }
    }
    while(1) {
        now = GET_CLOCK_TIME();
        if(ETS_CURRENT_SETTINGS.ewm_selector_type == EwmSelectorType::Switch || VEHICLE_CONFIG.shifter_style == 1) {
            ProfileSwitchPos prof_now = egs_can_hal->get_shifter_ws_mode(now, 100);
            // Switch based
            if (prof_now != last_mode) {
                last_mode = prof_now;
                if (prof_now != ProfileSwitchPos::SNV) {
                    gearbox->set_profile(profile_from_auto_ty(prof_now == ProfileSwitchPos::Top ? ETS_CURRENT_SETTINGS.profile_idx_top : ETS_CURRENT_SETTINGS.profile_idx_buttom)); 
                }
            }
        } else if (ETS_CURRENT_SETTINGS.ewm_selector_type == EwmSelectorType::Button) {
            // EWM button
            bool down = egs_can_hal->get_profile_btn_press(now, 100);
            if (down && !pressed) {
                // Pressed button, inc profile
                profile_id++;
                if (profile_id == NUM_PROFILES) {
                    profile_id = 0;
                }
                gearbox->set_profile(profiles[profile_id]);
            }
            pressed = down;
        } // Else - No profile button to process


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
        ShifterPosition spos = egs_can_hal->get_shifter_position(now, 1000);
        if (spos != slast_pos) { // Same position, ignore
            // Process last request of the user
            if (slast_pos == ShifterPosition::PLUS) {
                gearbox->inc_gear_request();
            } else if (slast_pos == ShifterPosition::MINUS) {
                gearbox->dec_gear_request();
            }
            slast_pos = spos;
        }
        vTaskDelay(20/portTICK_PERIOD_MS);
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
        case SPEAKER_POST_CODE::EFUSE_NOT_SET:
            return "EFUSE_CONFIG_NOT_SET";
        default:
            return nullptr;
    }
}

extern "C" void app_main(void)
{
    init_clock();
    // Set all pointers
    gearbox = nullptr;
    egs_can_hal = nullptr;
    pressure_manager = nullptr;
    SPEAKER_POST_CODE s = setup_tcm();
    xTaskCreate(err_beep_loop, "PCSPKR", 2048, reinterpret_cast<void*>(s), 2, nullptr);
    // Now spin up the KWP2000 server (last thing)
    diag_server = new Kwp2000_server(egs_can_hal, gearbox);
    xTaskCreatePinnedToCore(Kwp2000_server::start_kwp_server, "KWP2000", 32*1024, diag_server, 5, nullptr, 0);
    xTaskCreatePinnedToCore(Kwp2000_server::start_kwp_server_timer, "KWP2000TIMER", 4096, diag_server, 5, nullptr, 0);
    if (s != SPEAKER_POST_CODE::INIT_OK) {
        while(true) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "INIT", "TCM INIT ERROR (%s)! CANNOT START TCM!", post_code_to_str(s));
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    } else { // INIT OK!
        xTaskCreate(input_manager, "INPUT_MANAGER", 8192, nullptr, 5, nullptr);
    }
}

#endif // UNIT_TEST