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

Kwp2000_server *diag_server;

uint8_t profile_id = 0;

Speaker *spkr2 = nullptr;

Shifter *shifter = nullptr;

namespace {

constexpr uint8_t CAN_TX_PERIOD_MS = 20;
constexpr uint32_t CAN_BAUD = 500000;
constexpr uint32_t SOLENOID_BOOT_TEST_TIMEOUT_MS = 3000;

void create_fallback_can(Shifter* active_shifter)
{
    if (egs_can_hal != nullptr)
    {
        delete egs_can_hal;
        egs_can_hal = nullptr;
    }
    egs_can_hal = new EgsBaseCan("EGSBASIC", CAN_TX_PERIOD_MS, CAN_BAUD, active_shifter);
}

void enter_legacy_fallback_mode()
{
    pcb_gpio_matrix = nullptr;
    spkr = new Speaker(gpio_num_t::GPIO_NUM_4);  // Assume legacy when this fails
    spkr2 = new Speaker(gpio_num_t::GPIO_NUM_0); // For new PCBs
    create_fallback_can(nullptr);
}

bool init_gpio_matrix_from_board_ver()
{
    switch (BOARD_CONFIG.board_ver)
    {
    case 1:
        pcb_gpio_matrix = new BoardV11GpioMatrix();
        return true;
    case 2:
        pcb_gpio_matrix = new BoardV12GpioMatrix();
        return true;
    case 3:
        pcb_gpio_matrix = new BoardV13GpioMatrix();
        return true;
    default:
        return false;
    }
}

SPEAKER_POST_CODE init_runtime_services()
{
    if (ESP_OK != EEPROM::init_eeprom())
    {
        return SPEAKER_POST_CODE::EEPROM_FAIL;
    }

    if (ESP_OK != TCUIO::setup_io_layer())
    {
        CURRENT_DEVICE_MODE = DEVICE_MODE_ERROR;
        return SPEAKER_POST_CODE::SENSOR_FAIL;
    }

    if (ESP_OK != Solenoids::init_all_solenoids())
    {
        CURRENT_DEVICE_MODE = DEVICE_MODE_ERROR;
        return SPEAKER_POST_CODE::SOLENOID_FAIL;
    }

    CURRENT_DEVICE_MODE = EEPROM::read_device_mode();
    if (ESP_OK != EGSCal::init_egs_calibration())
    {
        CURRENT_DEVICE_MODE = DEVICE_MODE_NO_CALIBRATION;
        return SPEAKER_POST_CODE::CALIBRATION_FAIL;
    }

    ModuleConfiguration::load_all_settings();

    // Only now that the settings are loaded can the power on solenoid test run -
    // it compares against SOL_CURRENT_SETTINGS.current_threshold_error, and
    // starting it any earlier would test against the compiled in default.
    // Its verdict is consumed here, otherwise a solenoid drawing current when it
    // should be off would only ever be logged.
    if (ESP_OK != Solenoids::start_boot_test())
    {
        CURRENT_DEVICE_MODE = DEVICE_MODE_ERROR;
        return SPEAKER_POST_CODE::SOLENOID_FAIL;
    }
    if (!Solenoids::wait_for_boot_test(SOLENOID_BOOT_TEST_TIMEOUT_MS))
    {
        CURRENT_DEVICE_MODE = DEVICE_MODE_ERROR;
        return SPEAKER_POST_CODE::SOLENOID_FAIL;
    }

    Profiles::init_profiles(0 == VEHICLE_CONFIG.engine_type);
    return SPEAKER_POST_CODE::INIT_OK;
}

Shifter* create_shifter_from_settings()
{
    switch (VEHICLE_CONFIG.shifter_style)
    {
    case (uint8_t)ShifterStyle::EWM:
    case (uint8_t)ShifterStyle::SLR:
        return new ShifterEwm(&VEHICLE_CONFIG, &ETS_CURRENT_SETTINGS);
    case (uint8_t)ShifterStyle::TRRS:
        if (pcb_gpio_matrix->i2c_scl != GPIO_NUM_NC)
        {
            return new ShifterTrrs(&VEHICLE_CONFIG, pcb_gpio_matrix);
        }
        ESP_LOGE("PCB", "TRRS IS NOT COMPATIBLE WITH V1.1 PCB!");
        return nullptr;
    default:
        ESP_LOGE("INIT", "INVALID SHIFTER ID 0x%02X", VEHICLE_CONFIG.shifter_style);
        return nullptr;
    }
}

EgsBaseCan* create_vehicle_can_layer(Shifter* active_shifter)
{
    switch (VEHICLE_CONFIG.egs_can_type)
    {
    case 1:
        return new Egs51Can("EGS51", CAN_TX_PERIOD_MS, CAN_BAUD, active_shifter);
    case 2:
        return new Egs52Can("EGS52", CAN_TX_PERIOD_MS, CAN_BAUD, active_shifter);
    case 3:
        return new Egs53Can("EGS53", CAN_TX_PERIOD_MS, CAN_BAUD, active_shifter);
    case 4:
        // HFM only works with a TRRS shifter. egs_can_type and shifter_style are
        // configured independently, so casting whatever we were given would be a
        // bad downcast (eg. handing a ShifterEwm over as a ShifterTrrs).
        if (VEHICLE_CONFIG.shifter_style == (uint8_t)ShifterStyle::TRRS)
        {
            return new HfmCan("HFM", CAN_TX_PERIOD_MS, static_cast<ShifterTrrs*>(active_shifter));
        }
        ESP_LOGE("INIT", "HFM CAN requires a TRRS shifter, falling back to basic CAN (Diag only!)");
        return new EgsBaseCan("EGSBASIC", CAN_TX_PERIOD_MS, CAN_BAUD, active_shifter);
    case 5:
        return new CustomCan("CC", CAN_TX_PERIOD_MS, CAN_BAUD, active_shifter);
    default:
        ESP_LOGE("INIT", "ERROR. CAN Mode not set, falling back to basic CAN (Diag only!)");
        return new EgsBaseCan("EGSBASIC", CAN_TX_PERIOD_MS, CAN_BAUD, active_shifter);
    }
}

SPEAKER_POST_CODE init_drivetrain_runtime()
{
    shifter = create_shifter_from_settings();
    if (nullptr == shifter)
    {
        return SPEAKER_POST_CODE::CONFIGURATION_MISMATCH;
    }

    if (egs_can_hal != nullptr)
    {
        delete egs_can_hal;
        egs_can_hal = nullptr;
    }
    egs_can_hal = create_vehicle_can_layer(shifter);

    if (!egs_can_hal->begin_task())
    {
        return SPEAKER_POST_CODE::CAN_FAIL;
    }

    gearbox = new Gearbox(shifter);
    if (ESP_OK != gearbox->start_controller())
    {
        CURRENT_DEVICE_MODE = DEVICE_MODE_ERROR;
        return SPEAKER_POST_CODE::CONTROLLER_FAIL;
    }

    gearbox->set_profile(shifter->get_profile(50u));
    return SPEAKER_POST_CODE::INIT_OK;
}

void start_diag_server_tasks()
{
    diag_server = new Kwp2000_server(egs_can_hal, gearbox);
    xTaskCreatePinnedToCore(Kwp2000_server::start_kwp_server, "KWP2000", 16 * 1024, diag_server, 5, nullptr, 0);
    xTaskCreatePinnedToCore(Kwp2000_server::start_kwp_server_timer, "KWP2000TIMER", 1024, diag_server, 5, nullptr, 0);
}

} // namespace

SPEAKER_POST_CODE setup_tcm()
{
    CURRENT_DEVICE_MODE = DEVICE_MODE_NO_EFUSE;

    if (ESP_OK != EEPROM::read_efuse_config(&BOARD_CONFIG))
    {
        enter_legacy_fallback_mode();
        return SPEAKER_POST_CODE::EFUSE_NOT_SET;
    }

    if (!init_gpio_matrix_from_board_ver())
    {
        enter_legacy_fallback_mode();
        return SPEAKER_POST_CODE::EFUSE_NOT_SET;
    }

    spkr = new Speaker(pcb_gpio_matrix->spkr_pin);
    create_fallback_can(nullptr);

    SPEAKER_POST_CODE init_result = init_runtime_services();
    if (init_result != SPEAKER_POST_CODE::INIT_OK)
    {
        return init_result;
    }

    return init_drivetrain_runtime();
}

void err_beep_loop(void *a)
{
    SPEAKER_POST_CODE p = (SPEAKER_POST_CODE)(int)a;
    if (p == SPEAKER_POST_CODE::INIT_OK)
    {
        if (nullptr != spkr)
        {
            spkr->post(p); // All good, return
        }
        if (nullptr != egs_can_hal)
        {
            egs_can_hal->set_gearbox_ok(true);
        }
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
            if (nullptr != spkr)
            {
                spkr->post(p);
            }
            if (spkr2 != nullptr)
            {
                //spkr2->post(p);
            }
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
}

void input_manager()
{
    PaddlePosition last_pos = PaddlePosition::None;
    ShifterPosition slast_pos = ShifterPosition::SignalNotAvailable;
    while (1)
    {
        if (nullptr != ioexpander) {
            ioexpander->read_from_ioexpander();
        }
        if (nullptr != shifter && nullptr != gearbox && nullptr != egs_can_hal) {
            AbstractProfile* prof = shifter->get_profile(500);
            gearbox->set_profile(prof);
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
        }
        if (nullptr != pcb_gpio_matrix) {
            pcb_gpio_matrix->write_output_signals();
        }
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

    gearbox = nullptr;
    egs_can_hal = nullptr;
    pressure_manager = nullptr;

    SPEAKER_POST_CODE s = setup_tcm();
    xTaskCreate(err_beep_loop, "PCSPKR", 1024, reinterpret_cast<void*>(s), 2, nullptr);

    // Now spin up the KWP2000 server (last thing)
    start_diag_server_tasks();
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
        input_manager();
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