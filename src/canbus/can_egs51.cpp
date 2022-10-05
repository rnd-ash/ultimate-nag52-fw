#include "can_egs51.h"

#ifdef EGS51_MODE

#ifndef BOARD_V2
    #error "EGS51 CAN Support is only supported on V2 PCBs!"
#endif

#define IO_ADDR 0x20

#include "driver/twai.h"
#include "pins.h"
#include "gearbox_config.h"
#include "driver/i2c.h"

Egs51Can::Egs51Can(const char* name, uint8_t tx_time_ms)
    : AbstractCan(name, tx_time_ms)
{
    // Firstly try to init CAN
    ESP_LOG_LEVEL(ESP_LOG_INFO, "EGS51_CAN", "CAN constructor called");
    twai_general_config_t gen_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    gen_config.intr_flags = ESP_INTR_FLAG_IRAM; // Set TWAI interrupt to IRAM (Enabled in menuconfig)!
    gen_config.rx_queue_len = 32;
    gen_config.tx_queue_len = 32;
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t res;
    res = twai_driver_install(&gen_config, &timing_config, &filter_config);
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS51_CAN", "TWAI_DRIVER_INSTALL FAILED!: %s", esp_err_to_name(res));
    }
    res = twai_start();
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS51_CAN", "TWAI_START FAILED!: %s", esp_err_to_name(res));
    }

    // Init TRRS sensors
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = 400000;
    res = i2c_driver_install(I2C_NUM_0, i2c_mode_t::I2C_MODE_MASTER, 0, 0, 0);
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "I2C", "Failed to install driver %s", esp_err_to_name(res));
        return;
    }
    res = i2c_param_config(I2C_NUM_0, &conf);
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "I2C", "Failed to set param config %s", esp_err_to_name(res));
        return;
    }

    this->gs218.set_TORQUE_REQ(0xFE);
    this->gs218.bytes[7] = 0xFE;
    this->gs218.bytes[4] = 0x48;
    this->gs218.bytes[3] = 0x64;
    // CAN is OK!
    this->can_init_ok = true;
}

bool Egs51Can::begin_tasks() {
    if (!this->can_init_ok) { // Cannot init tasks if CAN is dead!
        return false;
    }
    // Prevent starting again
    if (this->rx_task == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "EGS51_CAN", "Starting CAN Rx task");
        if (xTaskCreate(this->start_rx_task_loop, "EGS51_CAN_RX", 8192, this, 5, this->rx_task) != pdPASS) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS51_CAN", "CAN Rx task creation failed!");
            return false;
        }
    }
    if (this->tx_task == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_INFO, "EGS51_CAN", "Starting CAN Tx task");
        if (xTaskCreate(this->start_tx_task_loop, "EGS51_CAN_TX", 4096, this, 5, this->tx_task) != pdPASS) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS51_CAN", "CAN Tx task creation failed!");
            return false;
        }
    }
    return true; // Ready!
}

Egs51Can::~Egs51Can()
{
    if (this->rx_task != nullptr) {
        vTaskDelete(this->rx_task);
    }
    if (this->tx_task != nullptr) {
        vTaskDelete(this->tx_task);
    }
    // Delete CAN
    if (this->can_init_ok) {
        twai_stop();
        twai_driver_uninstall();
    }
}

WheelData Egs51Can::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData Egs51Can::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvaliable
    };
}

WheelData Egs51Can::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_208 bs208;
    if (this->esp51.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvaliable;
        switch(bs208.get_DRTGHR()) {
            case BS_208h_DRTGHR::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHR::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHR::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHR::SNV:
            default:
                break;
        }

        return WheelData {
            .double_rpm = bs208.get_DHR(),
            .current_dir = d
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvaliable
        };
    }
}

WheelData Egs51Can::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_208 bs208;
    if (this->esp51.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvaliable;
        switch(bs208.get_DRTGHL()) {
            case BS_208h_DRTGHL::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHL::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHL::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHL::SNV:
            default:
                break;
        }

        return WheelData {
            .double_rpm = bs208.get_DHL(),
            .current_dir = d
        };
    } else {
        return WheelData {
            .double_rpm = 0,
            .current_dir = WheelDirection::SignalNotAvaliable
        };
    }
}

ShifterPosition Egs51Can::get_shifter_position_ewm(uint64_t now, uint64_t expire_time_ms) {
    if (now - this->last_i2c_query_time > expire_time_ms) {
        return ShifterPosition::SignalNotAvaliable;
    }
    // Data is valid time range!
    uint8_t tmp = this->i2c_rx_bytes[0];
    bool TRRS_A = (tmp & (uint8_t)BIT(5)) != 0;
    bool TRRS_B = (tmp & (uint8_t)BIT(6)) != 0;
    bool TRRS_C = (tmp & (uint8_t)BIT(3)) != 0;
    bool TRRS_D = (tmp & (uint8_t)BIT(4)) != 0;

    if (TRRS_A && TRRS_B && TRRS_C && !TRRS_D) {
        this->last_valid_position = ShifterPosition::P;
        return ShifterPosition::P;
    } else if (!TRRS_A && TRRS_B && TRRS_C && TRRS_D) {
        this->last_valid_position = ShifterPosition::R;
        return ShifterPosition::R;
    } else if (TRRS_A && !TRRS_B && TRRS_C && TRRS_D) {
        this->last_valid_position = ShifterPosition::N;
        return ShifterPosition::N;
    } else if (!TRRS_A && !TRRS_B && TRRS_C && !TRRS_D) {
        this->last_valid_position = ShifterPosition::D;
        return ShifterPosition::D;
    } else if (!TRRS_A && !TRRS_B && !TRRS_C && TRRS_D) {
        this->last_valid_position = ShifterPosition::FOUR;
        return ShifterPosition::FOUR;
    } else if (!TRRS_A && TRRS_B && !TRRS_C && !TRRS_D) {
        this->last_valid_position = ShifterPosition::THREE;
        return ShifterPosition::THREE;
    } else if (TRRS_A && !TRRS_B && !TRRS_C && !TRRS_D) {
        this->last_valid_position = ShifterPosition::TWO;
        return ShifterPosition::TWO;
    } else if (TRRS_A && TRRS_B && !TRRS_C && TRRS_D) {
        this->last_valid_position = ShifterPosition::ONE;
        return ShifterPosition::ONE;
    } else if (!TRRS_A && !TRRS_B && !TRRS_C && !TRRS_D) { // Intermediate position, now work out which one
        if (this->last_valid_position == ShifterPosition::P) {
            return ShifterPosition::P_R;
        } else if (this->last_valid_position == ShifterPosition::R) {
            return ShifterPosition::R_N;
        } else if (this->last_valid_position == ShifterPosition::D || this->last_valid_position == ShifterPosition::N) {
            return ShifterPosition::N_D;
        } else {
            return ShifterPosition::SignalNotAvaliable; // invalid combination
        }
    } else {
        return ShifterPosition::SignalNotAvaliable;
    }
}

EngineType Egs51Can::get_engine_type(uint64_t now, uint64_t expire_time_ms) {
    return EngineType::Unknown;
}

bool Egs51Can::get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs51Can::get_kickdown(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

uint8_t Egs51Can::get_pedal_value(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_210 ms210;
    if (this->ms51.get_MS_210(now, expire_time_ms, &ms210)) {
        return ms210.get_PW();
    } else {
        return 0xFF;
    }
}

int Egs51Can::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_310 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return (int)ms310.get_STA_TORQUE()*2;
    } else {
        return 0xFF;
    }
    return INT_MAX;
}

int Egs51Can::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_310 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return (int)ms310.get_MAX_TORQUE()*2;
    } else {
        return 0xFF;
    }
    return INT_MAX;
}

int Egs51Can::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    return 0; // Always 0 on W210 as ECUs do NOT calculate inertia
}

PaddlePosition Egs51Can::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    return PaddlePosition::SNV;
}

int16_t Egs51Can::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    MS_608 ms608;
    if (this->ms51.get_MS_608(now, expire_time_ms, &ms608)) {
        return ms608.get_T_MOT() - 40;
    } else {
        return UINT16_MAX;
    }
}

int16_t Egs51Can::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_308 ms308;
    if (this->ms51.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.get_T_OEL() - 40;
    } else {
        return UINT16_MAX;
    }
}

uint16_t Egs51Can::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    MS_308 ms308;
    if (this->ms51.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.get_NMOT();
    } else {
        return UINT16_MAX;
    }
}

bool Egs51Can::get_is_starting(uint64_t now, uint64_t expire_time_ms) { // TODO
    return false;
}

bool Egs51Can::get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

bool Egs51Can::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    return false;
}

void Egs51Can::set_clutch_status(ClutchStatus status) {
    
}

void Egs51Can::set_actual_gear(GearboxGear actual) {
    switch(actual) {
        case GearboxGear::First:
            this->gs218.set_GIC(GS_218h_GIC::G_D1);
            break;
        case GearboxGear::Second:
            this->gs218.set_GIC(GS_218h_GIC::G_D2);
            break;
        case GearboxGear::Third:
            this->gs218.set_GIC(GS_218h_GIC::G_D3);
            break;
        case GearboxGear::Fourth:
            this->gs218.set_GIC(GS_218h_GIC::G_D4);
            break;
        case GearboxGear::Fifth:
            this->gs218.set_GIC(GS_218h_GIC::G_D5);
            break;
        case GearboxGear::Park:
            this->gs218.set_GIC(GS_218h_GIC::G_P);
            break;
        case GearboxGear::Neutral:
            this->gs218.set_GIC(GS_218h_GIC::G_N);
            break;
        case GearboxGear::Reverse_First:
            this->gs218.set_GIC(GS_218h_GIC::G_R);
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.set_GIC(GS_218h_GIC::G_R2);
            break;
        case GearboxGear::SignalNotAvaliable:
        default:
            this->gs218.set_GIC(GS_218h_GIC::G_SNV);
            break;
    }
}

void Egs51Can::set_target_gear(GearboxGear target) {
    switch(target) {
        case GearboxGear::First:
            this->gs218.set_GZC(GS_218h_GZC::G_D1);
            break;
        case GearboxGear::Second:
            this->gs218.set_GZC(GS_218h_GZC::G_D2);
            break;
        case GearboxGear::Third:
            this->gs218.set_GZC(GS_218h_GZC::G_D3);
            break;
        case GearboxGear::Fourth:
            this->gs218.set_GZC(GS_218h_GZC::G_D4);
            break;
        case GearboxGear::Fifth:
            this->gs218.set_GZC(GS_218h_GZC::G_D5);
            break;
        case GearboxGear::Park:
            this->gs218.set_GZC(GS_218h_GZC::G_P);
            break;
        case GearboxGear::Neutral:
            this->gs218.set_GZC(GS_218h_GZC::G_N);
            break;
        case GearboxGear::Reverse_First:
            this->gs218.set_GZC(GS_218h_GZC::G_R);
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.set_GZC(GS_218h_GZC::G_R2);
            break;
        case GearboxGear::SignalNotAvaliable:
        default:
            this->gs218.set_GZC(GS_218h_GZC::G_SNV);
            break;
    }
}

void Egs51Can::set_safe_start(bool can_start) {
    this->start_enable = can_start;
}

void Egs51Can::set_race_start(bool race_start) {
}

void Egs51Can::set_gearbox_temperature(uint16_t temp) {
}

void Egs51Can::set_input_shaft_speed(uint16_t rpm) {
}

void Egs51Can::set_is_all_wheel_drive(bool is_4wd) {
}

void Egs51Can::set_wheel_torque(uint16_t t) {
}

void Egs51Can::set_shifter_position(ShifterPosition pos) {
}

void Egs51Can::set_gearbox_ok(bool is_ok) {
}

void Egs51Can::set_torque_request(TorqueRequest request) {
    if (request == TorqueRequest::None) {
        this->gs218.set_TORQUE_REQ_EN(false);
    } else {
        // Just enable the request
        this->gs218.set_TORQUE_REQ_EN(true);
    }
}

void Egs51Can::set_requested_torque(uint16_t torque_nm) {
    if (torque_nm == 0 && gs218.get_TORQUE_REQ_EN() == false) {
        this->gs218.set_TORQUE_REQ(0xFE);
    } else {
        this->gs218.set_TORQUE_REQ(torque_nm/2);
    }
}

void Egs51Can::set_error_check_status(SystemStatusCheck ssc) {
}


void Egs51Can::set_turbine_torque_loss(uint16_t loss_nm) {
}

void Egs51Can::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
}

void Egs51Can::set_drive_profile(GearboxProfile p) {
}

void Egs51Can::set_display_msg(GearboxMessage msg) {
}

void Egs51Can::set_wheel_torque_multi_factor(float ratio) {
}

/**
 * Parity calculation for torque numbers on GS218 and GS418
 * 
 * Each torque request member is a struct of 16 bits comprised of the following fields:
 * 1. Toggle bit (1 bit)
 * 2. Max request - bool (1 bit)
 * 3. Min request - bool (1 bit)
 * 4. Required torque - 13 bits
 */
inline bool calc_torque_parity(uint16_t s) {
    uint16_t p = s;
    p ^= (p >> 1);
    p ^= (p >> 2);
    p ^= (p >> 4);
    p ^= (p >> 8);
    return (p & 1) == 1;
}

inline void to_bytes(uint64_t src, uint8_t* dst) {
    for(uint8_t i = 0; i < 8; i++) {
        dst[7-i] = src & 0xFF;
        src >>= 8;
    }
}

[[noreturn]]
void Egs51Can::tx_task_loop() {
    twai_message_t tx;
    tx.data_length_code = 8; // Always
    GS_218 gs_218tx;
    uint8_t cvn_counter = 0;
    bool toggle = false;
    bool time_to_toggle = false;
    while(true) {
        // Copy current CAN frame values to here so we don't
        // accidentally modify parity calculations
        gs_218tx = {gs218.raw};

        // Firstly we have to deal with toggled bits!
        // As toggle bits need to be toggled every 40ms,
        // and egs52 Tx interval is 20ms,
        // we can achieve this with 2 booleans
        //gs_218tx.set_MTGL_EGS(toggle);
        // Now do parity calculations
        //gs_218tx.set_MPAR_EGS(calc_torque_parity(gs_218tx.raw >> 48));
        if (time_to_toggle) {
            toggle = !toggle;
        }
        time_to_toggle = !time_to_toggle;
        
        // Now set CVN Counter (Increases every frame)
        gs_218tx.set_FEHLER(cvn_counter);
        cvn_counter++;

        if (!this->send_messages) {
            vTaskDelay(50);
            continue;
        }

        // Now send CAN Data!
        vTaskDelay(1 / portTICK_PERIOD_MS);
        tx.identifier = GS_218_CAN_ID;
        tx.data_length_code = 6;
        to_bytes(gs_218tx.raw, tx.data);
        twai_transmit(&tx, 5);
        vTaskDelay(this->tx_time_ms / portTICK_PERIOD_MS);
    }
}

[[noreturn]]
void Egs51Can::rx_task_loop() {
    twai_message_t rx;
    twai_status_info_t can_status;
    uint64_t now = 0;
    uint64_t tmp;
    uint8_t i;
    this->last_i2c_query_time = 0;
    while(true) {
        now = (esp_timer_get_time()/1000);
        twai_get_status_info(&can_status);
        if (can_status.msgs_to_rx == 0) {
            vTaskDelay(4 / portTICK_PERIOD_MS); // Wait for buffer to have at least 1 frame
        } else { // We have frames, read them
            if (now > 2) {
                now -= 2;
            }
            for(uint8_t x = 0; x < can_status.msgs_to_rx; x++) { // Read all frames
                if (twai_receive(&rx, pdMS_TO_TICKS(0)) == ESP_OK && rx.data_length_code != 0) {
                    tmp = 0;
                    for(i = 0; i < rx.data_length_code; i++) {
                        tmp |= (uint64_t)rx.data[i] << (8*(7-i));
                    }

                    if (this->ms51.import_frames(tmp, rx.identifier, now)) {
                    } else if (this->esp51.import_frames(tmp, rx.identifier, now)) {
                    }
                }
            }
            vTaskDelay(2 / portTICK_PERIOD_MS); // Reset watchdog here
        }
        if (now - this->last_i2c_query_time > 50) {
            // Query I2C IO Expander
            uint8_t req[2] = {0,0};
            esp_err_t e = i2c_master_write_read_device(I2C_NUM_0, IO_ADDR, req, 1, this->i2c_rx_bytes, 2, 5);
            if (e != ESP_OK) {
                // Error, SNV
                ESP_LOGE("LS", "Could not query I2C: %s", esp_err_to_name(e));
            } else {
                //ESP_LOGI("EGS51_CAN", "I2C Reponse %02X %02X", this->i2c_rx_bytes[0], this->i2c_rx_bytes[1]);
                this->last_i2c_query_time = now;
            }
            
            // Set RP and Start pins on IO expander to be outputs
            // IO 0+1 - OUTPUT
            // IO 2-7 - INPUT
            uint8_t write_buffer[2] = {0x07,0xFF}; // Set IO (0x06 + port 1 (0x01))
            if (start_enable) {
                write_buffer[1] = write_buffer[1] & ~(BIT(1));
            }
            //write_buffer[1] = ~(BIT(0));// | BIT(1)); // Start_EN and 

            i2c_master_write_to_device(I2C_NUM_0, IO_ADDR, write_buffer, 2, 50);

            /*
            req[0] = 0x03;
            req[1] = 0x00;
            uint8_t resp_tmp[2] = {0,0};
            e = i2c_master_write_read_device(I2C_NUM_0, IO_ADDR, req, 1, resp_tmp, 2, 5);
            if (e != ESP_OK) {
                // Error, SNV
                ESP_LOGE("LS", "Could not query I2C: %s", esp_err_to_name(e));
            } else {
                ESP_LOGI("LS", "I: %02X %02X", resp_tmp[0], resp_tmp[1]);
            }
            */
        }
    }
}

#endif