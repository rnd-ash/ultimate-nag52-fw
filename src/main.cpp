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
#include "egs_calibration/calibration_structs.h"

// CAN LAYERS
#include "canbus/can_egs51.h"
#include "canbus/can_egs52.h"
#include "canbus/can_egs53.h"
#include "canbus/can_hfm.h"
#include "canbus/can_custom.h"

#include "board_config.h"
#include "nvs/device_mode.h"

// shifter modules
#include "shifter/shifter.h"
#include "shifter/shifter_ewm.h"
#include "shifter/shifter_trrs.h"

Kwp2000_server *diag_server;

uint8_t profile_id = 0;

Speaker *spkr2 = nullptr;

Shifter *shifter = nullptr;

SPEAKER_POST_CODE setup_tcm()
{
    SPEAKER_POST_CODE ret = SPEAKER_POST_CODE::INIT_OK; // OK by default
    if (ESP_OK == EEPROM::read_efuse_config(&BOARD_CONFIG))
    {
        // First thing to do, Configure the GPIO Pin matrix!
        switch (BOARD_CONFIG.board_ver)
        {
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
            spkr = new Speaker(gpio_num_t::GPIO_NUM_4);  // Assume legacy when this fails!
            spkr2 = new Speaker(gpio_num_t::GPIO_NUM_0); // For new PCBs
            ret = SPEAKER_POST_CODE::EFUSE_NOT_SET;
            break;
        }
        egs_can_hal = new EgsBaseCan("EGSBASIC", 20, 500000, nullptr);
        if (ret == SPEAKER_POST_CODE::INIT_OK)
        {
            spkr = new Speaker(pcb_gpio_matrix->spkr_pin);
            if (ESP_OK == EEPROM::init_eeprom())
            {
                // Read device mode!
                CURRENT_DEVICE_MODE = EEPROM::read_device_mode();
                // Load EGS Calibration
                if (ESP_OK == EGSCal::init_egs_calibration()) {
                    // Read our configuration (This is allowed to fail as the default opts are always set by default)
                    ModuleConfiguration::load_all_settings();
                    // init driving profiles
                    Profiles::init_profiles(0 == VEHICLE_CONFIG.engine_type);

                    // init the shifter module
                    switch (VEHICLE_CONFIG.shifter_style)
                    {
                    case (uint8_t)ShifterStyle::EWM:
                    case (uint8_t)ShifterStyle::SLR:
                        shifter = new ShifterEwm(&VEHICLE_CONFIG, &ETS_CURRENT_SETTINGS);
                        break;
                    case (uint8_t)ShifterStyle::TRRS:
                        if (pcb_gpio_matrix->i2c_scl != GPIO_NUM_NC) {
                            shifter = new ShifterTrrs(&VEHICLE_CONFIG, pcb_gpio_matrix);
                        } else {
                            ESP_LOGE("PCB", "TRRS IS NOT COMPATIBLE WITH V1.1 PCB!");
                            shifter = nullptr;
                        }
                        break;
                    default:
                        // possibly
                        break;
                    }
                    if (nullptr != shifter)
                    {
                        // init the CAN module
                        egs_can_hal->~EgsBaseCan();
                        free(egs_can_hal); // Delete fallback CAN
                        switch (VEHICLE_CONFIG.egs_can_type)
                        {
                        case 1:
                            egs_can_hal = new Egs51Can("EGS51", 20, 500000, shifter); // EGS51 CAN Abstraction layer
                            break;
                        case 2:
                            egs_can_hal = new Egs52Can("EGS52", 20, 500000, shifter); // EGS52 CAN Abstraction layer
                            break;
                        case 3:
                            egs_can_hal = new Egs53Can("EGS53", 20, 500000, shifter); // EGS53 CAN Abstraction layer
                            break;
                        case 4:
                            egs_can_hal = new HfmCan("HFM", 20, reinterpret_cast<ShifterTrrs*>(shifter)); // HFM CAN Abstraction layer
                            break;
                        case 5:
                            egs_can_hal = new CustomCan("CC", 20, 500000, shifter); // Custom CAN Abstraction layer
                            break;
                        default:
                            // Unknown (Fallback to basic CAN)
                            ESP_LOGE("INIT", "ERROR. CAN Mode not set, falling back to basic CAN (Diag only!)");
                            egs_can_hal = new EgsBaseCan("EGSBASIC", 20, 500000, shifter);
                            break;
                        }
                        if (egs_can_hal->begin_task())
                        {
                            if (ESP_OK == Sensors::init_sensors())
                            {
                                if (ESP_OK == Solenoids::init_all_solenoids())
                                {
                                    gearbox = new Gearbox(shifter);
                                    if (ESP_OK == gearbox->start_controller())
                                    {
                                        gearbox->set_profile(shifter->get_profile(50u));
                                    }
                                    else
                                    {
                                        CURRENT_DEVICE_MODE = DEVICE_MODE_ERROR;
                                        ret = SPEAKER_POST_CODE::CONTROLLER_FAIL;
                                    }
                                }
                                else
                                {
                                    CURRENT_DEVICE_MODE = DEVICE_MODE_ERROR;
                                    ret = SPEAKER_POST_CODE::SOLENOID_FAIL;
                                }
                            }
                            else
                            {
                                ret = SPEAKER_POST_CODE::SENSOR_FAIL;
                            }
                        }
                        else
                        {
                            ret = SPEAKER_POST_CODE::CAN_FAIL;
                        }
                    }
                    else
                    {
                        ret = SPEAKER_POST_CODE::CONFIGURATION_MISMATCH;
                    }
                } else {
                    CURRENT_DEVICE_MODE = DEVICE_MODE_NO_CALIBRATION;
                    ret = SPEAKER_POST_CODE::CALIBRATION_FAIL;
                }
            }
            else
            {
                ret = SPEAKER_POST_CODE::EEPROM_FAIL;
            }
        }
    }
    else
    {
        ret = SPEAKER_POST_CODE::EFUSE_NOT_SET;
    }
    return ret;
}

void err_beep_loop(void *a)
{
    SPEAKER_POST_CODE p = (SPEAKER_POST_CODE)(int)a;
    if (p == SPEAKER_POST_CODE::INIT_OK)
    {
        spkr->post(p); // All good, return
        egs_can_hal->set_gearbox_ok(true);
        vTaskDelete(NULL);
    }
    else
    {
        if (egs_can_hal != nullptr)
        {
            // An error has occurred
            // Set gearbox to F mode
            egs_can_hal->set_drive_profile(GearboxProfile::Failure);
            egs_can_hal->set_display_msg(GearboxMessage::VisitWorkshop);
            egs_can_hal->set_gearbox_ok(false);
        }
        while (1)
        {
            spkr->post(p);
            if (spkr2 != nullptr)
            {
                //spkr2->post(p);
            }
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
}

void input_manager(void *)
{
    PaddlePosition last_pos = PaddlePosition::None;
    ShifterPosition slast_pos = ShifterPosition::SignalNotAvailable;
    while (1)
    {
        if (ioexpander) {
            ioexpander->read_from_ioexpander();
        }
        AbstractProfile* prof = shifter->get_profile(500);
        if (nullptr != prof) {
            gearbox->set_profile(prof);
        }
        PaddlePosition paddle = egs_can_hal->get_paddle_position(100);
        if (last_pos != paddle)
        { // Same position, ignore
            if (last_pos != PaddlePosition::None)
            {
                // Process last request of the user
                if (last_pos == PaddlePosition::Plus)
                {
                    gearbox->inc_gear_request();
                }
                else if (last_pos == PaddlePosition::Minus)
                {
                    gearbox->dec_gear_request();
                }
            }
            last_pos = paddle;
        }
        ShifterPosition spos = shifter->get_shifter_position(1000);
        if (spos != slast_pos)
        { // Same position, ignore
            // Process last request of the user
            if (slast_pos == ShifterPosition::PLUS)
            {
                gearbox->inc_gear_request();
            }
            else if (slast_pos == ShifterPosition::MINUS)
            {
                gearbox->dec_gear_request();
            }
            slast_pos = spos;
        }
        pcb_gpio_matrix->write_output_signals();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

const char *post_code_to_str(SPEAKER_POST_CODE s)
{
    switch (s)
    {
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
    case SPEAKER_POST_CODE::CONFIGURATION_MISMATCH:
        return "CONFIGURATION_MISMATCH";
    case SPEAKER_POST_CODE::CALIBRATION_FAIL:
        return "NO_EGS_CALIBRATION";
    default:
        return nullptr;
    }
}

extern "C" void app_main(void)
{
    init_clock();
    esp_log_level_set("gpio", esp_log_level_t::ESP_LOG_NONE);
    // Set all pointers
    gearbox = nullptr;
    egs_can_hal = nullptr;
    pressure_manager = nullptr;
    SPEAKER_POST_CODE s = setup_tcm();
    xTaskCreate(err_beep_loop, "PCSPKR", 1024, reinterpret_cast<void*>(s), 2, nullptr);
    // Now spin up the KWP2000 server (last thing)
    diag_server = new Kwp2000_server(egs_can_hal, gearbox);
    xTaskCreatePinnedToCore(Kwp2000_server::start_kwp_server, "KWP2000", 16*1024, diag_server, 5, nullptr, 0);
    xTaskCreatePinnedToCore(Kwp2000_server::start_kwp_server_timer, "KWP2000TIMER", 1024, diag_server, 5, nullptr, 0);
    if (s != SPEAKER_POST_CODE::INIT_OK)
    {
        while (true)
        {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "INIT", "TCM INIT ERROR (%s)! CANNOT START TCM!", post_code_to_str(s));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    else
    { // INIT OK!
        xTaskCreate(input_manager, "INPUT_MANAGER", 8192, nullptr, 5, nullptr);
    }
}

/*

TEST MAIN FUNCTION FOR CALIBRATING IDLE TICK COUNT

#include "diag/perf_mon.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" void app_main(void)
{
    //init_clock();
    PerfMon::init_perfmon();
    while(true) {
        PerfMon::update_sample();
        vTaskDelay(1000);
    }
}
*/

#endif // UNIT_TEST