#include "can_egs51.h"

#define IO_ADDR 0x20

#include "driver/twai.h"
#include "gearbox_config.h"
#include "driver/i2c.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"

typedef struct {
    bool a;
    bool b;
    bool c;
    bool d;
    ShifterPosition pos;
} TRRSPos;

const static TRRSPos TRRS_SHIFTER_TABLE[8] = {
    TRRSPos { .a = 1, .b = 1, .c = 1, .d = 0, .pos = ShifterPosition::P },
    TRRSPos { .a = 0, .b = 1, .c = 1, .d = 1, .pos = ShifterPosition::R },
    TRRSPos { .a = 1, .b = 0, .c = 1, .d = 1, .pos = ShifterPosition::N },
    TRRSPos { .a = 0, .b = 0, .c = 1, .d = 0, .pos = ShifterPosition::D },
    TRRSPos { .a = 0, .b = 0, .c = 0, .d = 1, .pos = ShifterPosition::FOUR },
    TRRSPos { .a = 0, .b = 1, .c = 0, .d = 0, .pos = ShifterPosition::THREE },
    TRRSPos { .a = 1, .b = 0, .c = 0, .d = 0, .pos = ShifterPosition::TWO },
    TRRSPos { .a = 1, .b = 1, .c = 0, .d = 1, .pos = ShifterPosition::ONE },
};

enum class PCAReg : uint8_t {
    INPUT0 = 0,
    INPUT1 = 1,
    OUTPUT0 = 2,
    OUTPUT1 = 3,
    POLARITY0 = 4,
    POLARITY1 = 5,
    CONFIG0 = 6,
    CONFIG1 = 7
};

enum class PCAPort : uint8_t {
    Port0 = 0,
    Port1 = 1
};


#define RPSolenoidV12 0
#define StartEnableV12 1
// On 1.3 boards, the RP and Start solenoid are flipped!
#define RPSolenoidV13 1
#define StartEnableV13 2//0
// Only on 1.3
#define GenMosfet 2

Egs51Can::Egs51Can(const char* name, uint8_t tx_time_ms, uint32_t baud) : EgsBaseCan(name, tx_time_ms, baud) {
    ESP_LOGI("EGS51", "SETUP CALLED");
    if (pcb_gpio_matrix->i2c_sda == gpio_num_t::GPIO_NUM_NC || pcb_gpio_matrix->i2c_scl == gpio_num_t::GPIO_NUM_NC) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "EGS51_CAN", "Cannot launch TRRS on board without I2C!");
        this->can_init_status = ESP_ERR_INVALID_VERSION;
    }
    if (this->can_init_status == ESP_OK) {
        // Init TRRS sensors
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = pcb_gpio_matrix->i2c_sda,
            .scl_io_num = pcb_gpio_matrix->i2c_scl,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
        };
        conf.master.clk_speed = 100000;
        this->can_init_status = i2c_driver_install(I2C_NUM_0, i2c_mode_t::I2C_MODE_MASTER, 0, 0, 0);
        if (this->can_init_status == ESP_OK) {
            this->can_init_status = i2c_param_config(I2C_NUM_0, &conf);
            if (this->can_init_status != ESP_OK) {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, this->name, "Failed to set param config");
            } else {
                uint8_t write_buffer[2] = {(uint8_t)PCAReg::CONFIG1, 0x00}; // Set IO 1 as Output
                this->can_init_status = i2c_master_write_to_device(I2C_NUM_0, IO_ADDR, write_buffer, 2, 50);
                if (this->can_init_status != ESP_OK) {
                    ESP_LOG_LEVEL(ESP_LOG_ERROR, this->name, "Failed to set output reg");
                } else {
                    write_buffer[0] = (uint8_t)PCAReg::CONFIG0; // Set IO 0
                    write_buffer[1] = 0xFF; // As all inputs
                    this->can_init_status = i2c_master_write_to_device(I2C_NUM_0, IO_ADDR, write_buffer, 2, 50);
                    if (this->can_init_status != ESP_OK) {
                        ESP_LOG_LEVEL(ESP_LOG_ERROR, this->name, "Failed to set input reg");
                    }
                }
            }
        } else {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, this->name, "Failed to install driver");
        }
    }

    this->start_enable = true;
    this->gs218.set_TORQUE_REQ(0xFE);
    this->gs218.bytes[7] = 0xFE;
    this->gs218.bytes[4] = 0x48;
    this->gs218.bytes[3] = 0x64;
}

WheelData Egs51Can::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {  // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData Egs51Can::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) { // TODO
    return WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
}

WheelData Egs51Can::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_208EGS51 bs208;
    if (this->esp51.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvailable;
        switch(bs208.get_DRTGHR()) {
            case BS_208h_DRTGHREGS51::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHREGS51::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHREGS51::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHREGS51::SNV:
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
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

WheelData Egs51Can::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    BS_208EGS51 bs208;
    if (this->esp51.get_BS_208(now, expire_time_ms, &bs208)) {
        WheelDirection d = WheelDirection::SignalNotAvailable;
        switch(bs208.get_DRTGHL()) {
            case BS_208h_DRTGHLEGS51::FWD:
                d = WheelDirection::Forward;
                break;
            case BS_208h_DRTGHLEGS51::REV:
                d = WheelDirection::Reverse;
                break;
            case BS_208h_DRTGHLEGS51::PASSIVE:
                d = WheelDirection::Stationary;
                break;
            case BS_208h_DRTGHLEGS51::SNV:
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
            .current_dir = WheelDirection::SignalNotAvailable
        };
    }
}

ShifterPosition Egs51Can::get_shifter_position_ewm(uint64_t now, uint64_t expire_time_ms) {
    ShifterPosition ret = ShifterPosition::SignalNotAvailable;
    if (VEHICLE_CONFIG.shifter_style == SHIFTER_STYLE_EWM) {
        EWM_230 dest;
        if (this->ewm.get_EWM_230(now, expire_time_ms, &dest)) {
            switch (dest.get_WHC()) {
                case EWM_230h_WHC::D:
                    ret = ShifterPosition::D;
                    break;
                case EWM_230h_WHC::N:
                    ret = ShifterPosition::N;
                    break;
                case EWM_230h_WHC::R:
                    ret = ShifterPosition::R;
                    break;
                case EWM_230h_WHC::P:
                    ret = ShifterPosition::P;
                    break;
                case EWM_230h_WHC::PLUS:
                    ret =  ShifterPosition::PLUS;
                    break;
                case EWM_230h_WHC::MINUS:
                    ret = ShifterPosition::MINUS;
                    break;
                case EWM_230h_WHC::N_ZW_D:
                    ret = ShifterPosition::N_D;
                    break;
                case EWM_230h_WHC::R_ZW_N:
                    ret = ShifterPosition::R_N;
                    break;
                case EWM_230h_WHC::P_ZW_R:
                    ret = ShifterPosition::P_R;
                    break;
                case EWM_230h_WHC::SNV:
                default:
                    break;
            }
        }
    } else {
        if (now - this->last_i2c_query_time < expire_time_ms) {
            // Data is valid time range!
            uint8_t tmp = this->i2c_rx_bytes[0];
            bool TRRS_A;
            bool TRRS_B;
            bool TRRS_C;
            bool TRRS_D;
            if (BOARD_CONFIG.board_ver == 2) { // V1.2 layout
                TRRS_A = (tmp & (uint8_t)BIT(5)) != 0;
                TRRS_B = (tmp & (uint8_t)BIT(6)) != 0;
                TRRS_C = (tmp & (uint8_t)BIT(3)) != 0;
                TRRS_D = (tmp & (uint8_t)BIT(4)) != 0;
            } else { // V1.3+ layout
                TRRS_A = (tmp & (uint8_t)BIT(5)) != 0;
                TRRS_B = (tmp & (uint8_t)BIT(6)) != 0;
                TRRS_C = (tmp & (uint8_t)BIT(4)) != 0;
                TRRS_D = (tmp & (uint8_t)BIT(3)) != 0;
            }

            if (!TRRS_A && !TRRS_B && !TRRS_C && !TRRS_D) { // Intermediate position, now work out which one
                if (this->last_valid_position == ShifterPosition::P) {
                    ret = ShifterPosition::P_R;
                } else if (this->last_valid_position == ShifterPosition::R) {
                    ret = ShifterPosition::R_N;
                } else if (this->last_valid_position == ShifterPosition::D || this->last_valid_position == ShifterPosition::N) {
                    ret = ShifterPosition::N_D;
                }
            } else {
                // Check truth table
                for (uint8_t i = 0; i < 8; i++) {
                    TRRSPos pos = TRRS_SHIFTER_TABLE[i];
                    if (pos.a == TRRS_A && pos.b == TRRS_B && pos.c == TRRS_C && pos.d == TRRS_D) {
                        ret = pos.pos;
                        break;
                    }
                }
            }
        }
    }
    return ret;
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
    MS_210EGS51 ms210;
    if (this->ms51.get_MS_210(now, expire_time_ms, &ms210)) {
        return ms210.get_PW();
    } else {
        return 0xFF;
    }
}

int Egs51Can::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_310EGS51 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return (int)ms310.get_IND_TORQUE()*3;
    } else {
        return INT_MAX;
    }
    return INT_MAX;
}

int Egs51Can::get_driver_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    return this->get_static_engine_torque(now, expire_time_ms);
}

int Egs51Can::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_310EGS51 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return (int)ms310.get_MAX_TORQUE()*3;
    } else {
        return INT_MAX;
    }
    return INT_MAX;
}

int Egs51Can::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    MS_310EGS51 ms310;
    if (this->ms51.get_MS_310(now, expire_time_ms, &ms310)) {
        return (int)ms310.get_MIN_TORQUE()*3;
    } else {
        return INT_MAX;
    }
    return INT_MAX;
}

PaddlePosition Egs51Can::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    return PaddlePosition::SNV;
}

int16_t Egs51Can::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    MS_608EGS51 ms608;
    if (this->ms51.get_MS_608(now, expire_time_ms, &ms608)) {
        return ms608.get_T_MOT() - 40;
    } else {
        return UINT16_MAX;
    }
}

int16_t Egs51Can::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) { // TODO
    MS_308EGS51 ms308;
    if (this->ms51.get_MS_308(now, expire_time_ms, &ms308)) {
        return ms308.get_T_OEL() - 40;
    } else {
        return UINT16_MAX;
    }
}

uint16_t Egs51Can::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    MS_308EGS51 ms308;
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
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D1);
            break;
        case GearboxGear::Second:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D2);
            break;
        case GearboxGear::Third:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D3);
            break;
        case GearboxGear::Fourth:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D4);
            break;
        case GearboxGear::Fifth:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_D5);
            break;
        case GearboxGear::Park:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_P);
            break;
        case GearboxGear::Neutral:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_N);
            break;
        case GearboxGear::Reverse_First:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_R);
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_R2);
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.set_GIC(GS_218h_GICEGS51::G_SNV);
            break;
    }
}

void Egs51Can::set_target_gear(GearboxGear target) {
    switch(target) {
        case GearboxGear::First:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D1);
            break;
        case GearboxGear::Second:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D2);
            break;
        case GearboxGear::Third:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D3);
            break;
        case GearboxGear::Fourth:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D4);
            break;
        case GearboxGear::Fifth:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_D5);
            break;
        case GearboxGear::Park:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_P);
            break;
        case GearboxGear::Neutral:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_N);
            break;
        case GearboxGear::Reverse_First:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_R);
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_R2);
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.set_GZC(GS_218h_GZCEGS51::G_SNV);
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

void Egs51Can::set_torque_request(TorqueRequest request, float amount_nm) {
    if (request == TorqueRequest::None) {
        this->gs218.set_TORQUE_REQ_EN(false);
        this->gs218.set_TORQUE_REQ(0xFE);
    } else {
        // Just enable the request
        this->gs218.set_TORQUE_REQ_EN(true);
        this->gs218.set_TORQUE_REQ(amount_nm/2);
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

/**
 * @brief void tx_frames() override;
        void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) override;
        void on_rx_done(uint64_t now_ts) override;
 * 
 */

void Egs51Can::tx_frames() {
    twai_message_t tx;
    tx.data_length_code = 8; // Always
    GS_218EGS51 gs_218tx;
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
    tx.identifier = GS_218EGS51_CAN_ID;
    tx.data_length_code = 6;
    to_bytes(gs_218tx.raw, tx.data);
    twai_transmit(&tx, 5);
}

void Egs51Can::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) {
    if (this->ms51.import_frames(data, id, timestamp)) {
    } else if (this->esp51.import_frames(data, id, timestamp)) {
    } else if (this->ewm.import_frames(data, id, timestamp)) {
    }
}

void Egs51Can::on_rx_done(uint64_t now_ts) {
    if (now_ts - this->last_i2c_query_time > 50) {
        // Query I2C IO Expander
        uint8_t req[2] = {0,0};
        esp_err_t e = i2c_master_write_read_device(I2C_NUM_0, IO_ADDR, req, 1, this->i2c_rx_bytes, 2, 5);
        if (e != ESP_OK) {
            // Error, SNV
            ESP_LOGE("LS", "Could not query I2C: %s", esp_err_to_name(e));
        } else {
            //ESP_LOGI("EGS51_CAN", "I2C Reponse %02X %02X", this->i2c_rx_bytes[0], this->i2c_rx_bytes[1]);
            this->last_i2c_query_time = now_ts;
        }
        
        // Set RP and Start pins on IO expander to be outputs
        // IO 0+1 - OUTPUT
        // IO 2-7 - INPUT
        uint8_t write_buffer[2] = {(uint8_t)PCAReg::OUTPUT1, 0x00}; // Set IO (0x06 + port 1 (0x01))
        if (start_enable) {
            uint8_t bit = BOARD_CONFIG.board_ver == 2 ? StartEnableV12 : StartEnableV13;
            write_buffer[1] = write_buffer[1] | (BIT(bit));
        }
        e = i2c_master_write_to_device(I2C_NUM_0, IO_ADDR, write_buffer, 2, 50);
        if (e != ESP_OK) {
            ESP_LOGE("LS", "Could not send I2C: %s", esp_err_to_name(e));
        }
    }
}