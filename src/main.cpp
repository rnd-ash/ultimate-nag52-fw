#ifndef UNIT_TEST

#include "clock.hpp"
#include "embed_data.h"

#include "solenoids/solenoids.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "speaker.h"
#include "tcu_io/tcu_io.hpp"
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

#include "inputcomponents/kickdownswitch.hpp"

Kwp2000_server *diag_server;

uint8_t profile_id = 0;

Speaker *spkr2 = nullptr;

SPEAKER_POST_CODE setup_tcm()
{
    SPEAKER_POST_CODE ret = SPEAKER_POST_CODE::INIT_OK; // OK by default
    CURRENT_DEVICE_MODE = DEVICE_MODE_NO_EFUSE;
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
        egs_can_hal = new EgsBaseCan("EGSBASIC", 20, 500000);
        if (ret == SPEAKER_POST_CODE::INIT_OK)
        {
            spkr = new Speaker(pcb_gpio_matrix->spkr_pin);
            if (ESP_OK == EEPROM::init_eeprom())
            {
                if (ESP_OK == TCUIO::setup_io_layer())
                {
                    if (ESP_OK == Solenoids::init_all_solenoids())
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
                                ESP_LOGE("INIT", "INVALID SHIFTER ID 0x%02X",VEHICLE_CONFIG.shifter_style);
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
                                    egs_can_hal = new Egs51Can("EGS51", 20, 500000); // EGS51 CAN Abstraction layer
                                    break;
                                case 2:
                                    egs_can_hal = new Egs52Can("EGS52", 20, 500000); // EGS52 CAN Abstraction layer
                                    break;
                                case 3:
                                    egs_can_hal = new Egs53Can("EGS53", 20, 500000); // EGS53 CAN Abstraction layer
                                    break;
                                case 4:
                                    egs_can_hal = new HfmCan("HFM", 20); // HFM CAN Abstraction layer
                                    break;
                                case 5:
                                    egs_can_hal = new CustomCan("CC", 20, 500000); // Custom CAN Abstraction layer
                                    break;
                                default:
                                    // Unknown (Fallback to basic CAN)
                                    ESP_LOGE("INIT", "ERROR. CAN Mode not set, falling back to basic CAN (Diag only!)");
                                    egs_can_hal = new EgsBaseCan("EGSBASIC", 20, 500000);
                                    break;
                                }
                                if (egs_can_hal->begin_task())
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

inline void set_start_enable(void){
    bool is_safe_start = gearbox->is_safe_start();
    egs_can_hal->set_safe_start(is_safe_start);
    if (ioexpander != nullptr) {
        ioexpander->set_start(is_safe_start);
    }
}

void input_manager(void *)
{
    const uint32_t expire_time = 5000u;
    PaddlePosition paddle_pos_last = PaddlePosition::None;
    ShifterPosition shifter_pos_last = ShifterPosition::SignalNotAvailable;
    ShifterPosition spos;
    while (1)
    {
        pcb_gpio_matrix->read_input_signals();
        AbstractProfile* prof = shifter->get_profile(expire_time);
        if (nullptr != prof) {
            gearbox->set_profile(prof);
        }
        PaddlePosition paddle = egs_can_hal->get_paddle_position(100u);
        if (paddle_pos_last != paddle)
        {
            // Same position is ignored
            // Process last request of the user
            switch (paddle_pos_last)
            {
            case PaddlePosition::Plus:
                gearbox->inc_gear_request();
                break;
            case PaddlePosition::Minus:
                gearbox->dec_gear_request();
                break;
            default:
                break;
            }
            paddle_pos_last = paddle;
        }
        // egs_can_hal->get_engine_iat_temp(expire_time);
        egs_can_hal->get_engine_coolant_temp(expire_time);
        spos = shifter->get_shifter_position(expire_time);
        if((shifter->get_shifter_type() == ShifterStyle::EWM) || (shifter->get_shifter_type() == ShifterStyle::SLR)) {
            if (spos != shifter_pos_last)
            {
                // Same position, ignore
                // Process last request of the user
                switch (shifter_pos_last)
                {
                case ShifterPosition::PLUS:
                    gearbox->inc_gear_request();
                    break;
                case ShifterPosition::MINUS:
                    gearbox->dec_gear_request();
                    break;
                default:
                    break;
                }
                shifter_pos_last = spos;
            }
        }
        set_start_enable();
        if (KickdownSwitch::is_kickdown_newly_pressed(egs_can_hal, expire_time) && (!egs_can_hal->get_engine_is_limp(expire_time))) {
            gearbox->dec_gear_request();            
        }
        pcb_gpio_matrix->write_output_signals();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

const char *post_code_to_str(SPEAKER_POST_CODE s)
{
    const char* ret = nullptr;
    switch (s)
    {
    case SPEAKER_POST_CODE::INIT_OK:
        ret = "INIT_OK";
        break;
    case SPEAKER_POST_CODE::CAN_FAIL:
        ret = "CAN_INIT_FAIL";
        break;
    case SPEAKER_POST_CODE::CONTROLLER_FAIL:
        ret = "CONTROLLER_INIT_FAIL";
        break;
    case SPEAKER_POST_CODE::EEPROM_FAIL:
        ret = "ERRPOM_INIT_FAIL";
        break;
    case SPEAKER_POST_CODE::SENSOR_FAIL:
        ret = "SENSOR_INIT_FAIL";
        break;
    case SPEAKER_POST_CODE::SOLENOID_FAIL:
        ret = "SOLENOID_INIT_FAIL";
        break;
    case SPEAKER_POST_CODE::EFUSE_NOT_SET:
        ret = "EFUSE_CONFIG_NOT_SET";
        break;
    case SPEAKER_POST_CODE::CONFIGURATION_MISMATCH:
        ret = "CONFIGURATION_MISMATCH";
        break;
    case SPEAKER_POST_CODE::CALIBRATION_FAIL:
        ret = "NO_EGS_CALIBRATION";
        break;
    default:
        break;
    }
    return ret;
}

extern "C" void app_main(void)
{
    esp_log_level_set("gpio", esp_log_level_t::ESP_LOG_NONE);
    // Set all pointers
    gearbox = nullptr;
    egs_can_hal = nullptr;
    pressure_manager = nullptr;
    SPEAKER_POST_CODE s = setup_tcm();
    xTaskCreate(err_beep_loop, "PCSPKR", 1024, reinterpret_cast<void*>(s), 2, nullptr);
    
    int x = (int)embed_container_end - (int)embed_container_start;
    printf("Embedded container: %p %p - %d Bytes\n", embed_container_start, embed_container_end, x);
    
    // Now spin up the KWP2000 server (last thing)
    diag_server = new Kwp2000_server(egs_can_hal, gearbox, shifter);
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