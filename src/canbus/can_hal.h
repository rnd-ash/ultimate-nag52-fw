
/**
 * CANBUS abstraction layer for EGS52 AND EGS53!
 */

#ifndef __ABSTRACT_CAN_H_
#define __ABSTRACT_CAN_H_

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <freertos/queue.h>
#include <string.h>
#include "driver/twai.h"

enum class WheelDirection: uint8_t {
    Forward, // Wheel going forwards
    Reverse, // Wheel going backwards
    Stationary, // Stationary (Not forward or backwards)
    SignalNotAvailable = 0xFF // SNV
};

struct WheelData {
    int double_rpm; // 2x real RPM (Better accuracy)
    WheelDirection current_dir; // Wheel direction
};

enum class SystemStatusCheck: uint8_t {
    /// @brief Waiting for check to complete
    Waiting,
    /// @brief Error check OK
    OK,
    /// @brief Error check failed
    Error
};

enum class EngineType: uint8_t {
    /// @brief Diesel engine
    Diesel,
    /// @brief Petrol engine
    Petrol,
    /// @brief Unknown engine type
    Unknown = 0xFF
};

/// @brief Torque request type
enum class TorqueRequest: uint8_t {
    /// @brief No torque request
    None,
    /// @brief Make less than specified amount
    LessThan,
    /// @brief Make more than specified amount
    MoreThan,
    /// @brief Make exactly specified amount
    Exact,
    /// @brief Make less than specified amount - Fast reaction
    LessThanFast,
    /// @brief Make more than specified amount - Fast reaction
    MoreThanFast,
    /// @brief Make exactly specified amount - Fast reaction
    ExactFast
};

/// @brief Gearbox gears for 722.6 gearbox
enum class GearboxGear: uint8_t {
    /// @brief Gear D1
    First = 1,
    /// @brief Gear D2
    Second = 2,
    /// @brief Gear D3
    Third = 3,
    /// @brief Gear D4
    Fourth = 4,
    /// @brief Gear D5
    Fifth = 5,
    /// @brief  Park
    Park = 8,
    /// @brief Neutral
    Neutral = 9,
    /// @brief Gear R1
    Reverse_First = 10,
    /// @brief Gear R2
    Reverse_Second = 11,
    /// @brief  Implausible or signal not available
    SignalNotAvailable = 0xFF
};

enum class PaddlePosition: uint8_t {
    None,
    Plus,
    Minus,
    PlusAndMinus,
    SNV = 0xFF
};

enum class ClutchStatus: uint8_t {
    Open,
    Slipping,
    Closed
};

enum class TransferCaseState: uint8_t {
    Hi, // High range
    Low, // Low range
    Neither, // Neutral
    Switching, // Switching in progress
    SNA = 0xFF, // Error
};
 
enum class GearboxProfile: uint8_t {
    // Comfort (C)
    Comfort,
    // Manual (M)
    Manual,
    // Agility (A)
    Agility,
    // Standard (S)
    Standard,
    // Winter (W)
    Winter,
    // Failure (F)
    Failure,
    // Race (R)
    Race,
    Individual,
    // Reserved for 'no profile' (_)
    Underscore
};

enum class ShifterPosition: uint8_t {
    P,
    P_R,
    R,
    R_N,
    N,
    N_D,
    D,
    PLUS, // For EWM only
    MINUS, // For EWM only
    FOUR, // For TRRS only
    THREE, // For TRRS only
    TWO, // For TRRS only
    ONE, // For TRRS only
    SignalNotAvailable = 0xFF // SNV
};

enum class SolenoidName: uint8_t {
    Y3,
    Y4,
    Y5,
    SPC,
    MPC,
    TCC
};

enum class GearboxDisplayGear: uint8_t {
    P,
    R,
    N,
    D,
    A,
    One,
    Two,
    Three,
    Four,
    Five,
    Failure,
    SNA = 0xFF,
};

enum class GearboxMessage: uint8_t {
    // No message
    None,
    // "Activate gearbox, parking brake!" with warning tone
    ActuateParkingBreak,
    // "Gearbox, shift lever according to N!"
    ShiftLeverToN,
    // "Activate gear, brake!"
    ActivateGear,
    // "Request gearbox, gear again!"
    RequestGearAgain,
    // "Transmission, Insert to Start N!"
    InsertToNToStart,
    // '^' indicator 
    Upshift,
    // 'v' indicator
    Downshift,
    // "Gearbox, visit workshop!"
    VisitWorkshop
};

enum class TerminalStatus {
    On,
    Off,
    SNA
};

const WheelData DEFAULT_SNV_WD {
    .double_rpm = 0,
    .current_dir = WheelDirection::SignalNotAvailable
};

class EgsBaseCan {
    public:
        EgsBaseCan(const char* name, uint8_t tx_time_ms, uint32_t baud);
        ~EgsBaseCan();
        bool begin_tasks();
        esp_err_t init_state() const;

        /**
         * Getters
         */

        // Get the front right wheel data
        virtual WheelData get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {
            return DEFAULT_SNV_WD;
        }
        // Get the front left wheel data
        virtual WheelData get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) {
            return DEFAULT_SNV_WD;
        }
        // Get the rear right wheel data
        virtual WheelData get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
            return DEFAULT_SNV_WD;
        }
        // Get the rear left wheel data
        virtual WheelData get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
            return DEFAULT_SNV_WD;
        }
        // Gets shifter position from EWM module
        virtual ShifterPosition get_shifter_position_ewm(uint64_t now, uint64_t expire_time_ms) {
            return ShifterPosition::SignalNotAvailable;
        }
        // Gets engine type
        virtual EngineType get_engine_type(uint64_t now, uint64_t expire_time_ms) {
            return EngineType::Unknown;
        }
        // Returns true if engine is in limp mode
        virtual bool get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) {
            return false;
        }
        // Returns true if pedal is kickdown 
        virtual bool get_kickdown(uint64_t now, uint64_t expire_time_ms) {
            return 0;
        }
        // Returns the pedal percentage. Range 0-250
        virtual uint8_t get_pedal_value(uint64_t now, uint64_t expire_time_ms) {
            return 0;
        }
        // Gets the current 'static' torque produced by the engine
        virtual int get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) {
            return 0;
        }
        // Gets driver torque demand (Default torque)
        virtual int get_driver_engine_torque(uint64_t now, uint64_t expire_time_ms) {
            return 0;
        }
        // Gets the maximum engine torque allowed at this moment by the engine map
        virtual int get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) {
            return 0;
        }
        // Gets the minimum engine torque allowed at this moment by the engine map
        virtual int get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) {
            return 0;
        }
        // Gets the torque loss of the AC system
        virtual uint8_t get_ac_torque_loss(uint64_t now, uint64_t expire_time_ms) {
            return UINT8_MAX;
        }

        // Gets the flappy paddle position
        virtual PaddlePosition get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
            return PaddlePosition::SNV;
        }
        // Gets engine coolant temperature
        virtual int16_t get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
            return INT16_MAX;
        }
        // Gets engine oil temperature
        virtual int16_t get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) {
            return INT16_MAX;
        }
        // Gets engine RPM
        virtual uint16_t get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
            return UINT16_MAX;
        }
        // Returns true if engine is cranking
        virtual bool get_is_starting(uint64_t now, uint64_t expire_time_ms) {
            return false;
        }
        virtual bool get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
            return false;
        }
        virtual bool get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) {
            return false;
        }
        virtual uint16_t get_fuel_flow_rate(uint64_t now, uint64_t expire_time_ms) {
            return 0;
        }
        // Gets status of terminal 15
        virtual TerminalStatus get_terminal_15(uint64_t now, uint64_t expire_time_ms) {
            return TerminalStatus::On; // Enabled by default unless implemented
        }

        virtual TransferCaseState get_transfer_case_state(uint64_t now, uint64_t expire_time_ms) {
            return TransferCaseState::SNA;
        }

        /**
         * Setters
         */

        virtual void set_race_start(bool race_start){};
        // Set solenoid PMW
        virtual void set_solenoid_pwm(uint16_t duty, SolenoidName s){};
        // Set the gearbox clutch position on CAN
        virtual void set_clutch_status(ClutchStatus status){};
        // Set the actual gear of the gearbox
        virtual void set_actual_gear(GearboxGear actual){};
        // Set the target gear of the gearbox
        virtual void set_target_gear(GearboxGear target){};
        // Sets the status bit indicating the car is safe to start
        virtual void set_safe_start(bool can_start){};
        // Sets the gerabox ATF temperature. Offset by +50C
        virtual void set_gearbox_temperature(uint16_t temp){};
        // Sets the RPM of the input shaft of the gearbox on CAN
        virtual void set_input_shaft_speed(uint16_t rpm){};
        // Sets 4WD activated toggle bit
        virtual void set_is_all_wheel_drive(bool is_4wd){};
        // Sets wheel torque
        virtual void set_wheel_torque(uint16_t t){};
        // Sets shifter position message
        virtual void set_shifter_position(ShifterPosition pos){};
        // Sets gearbox is OK
        virtual void set_gearbox_ok(bool is_ok){};
        // Sets torque request toggle
        virtual void set_torque_request(TorqueRequest request, int16_t amount_nm){};
        // Sets torque loss of torque converter
        virtual void set_turbine_torque_loss(uint16_t loss_nm){};
        // Sets torque multiplier factor from Engine all the way to wheels 
        virtual void set_wheel_torque_multi_factor(float ratio){};
        // Sets the status of system error check
        virtual void set_error_check_status(SystemStatusCheck ssc){};
        // Sets display profile
        virtual void set_display_gear(GearboxDisplayGear g, bool manual_mode){};
        // Sets drive profile
        virtual void set_drive_profile(GearboxProfile p){};
        // Sets display message
        virtual void set_display_msg(GearboxMessage msg){};
        // Set bit to signify the gearbox is aborting the shift
        virtual void set_abort_shift(bool is_aborting){};
        
        /// Custom setters
        virtual void set_spc_pressure(uint16_t p){}
        virtual void set_mpc_pressure(uint16_t p){}
        virtual void set_tcc_pressure(uint16_t p){}
        virtual void set_shift_stage(uint8_t stage, bool is_ramp){}
        virtual void set_gear_disagree(uint8_t count){}
        virtual void set_gear_ratio(int16_t g100){};

        // For diagnostic passive mode
        void enable_normal_msg_transmission() {
            this->send_messages = true;
        }

        // For diagnostic passive mode
        void disable_normal_msg_transmission() {
            this->send_messages = false;
        }

        // For diagnostics
        void register_diag_queue(QueueHandle_t* rx_queue, uint16_t rx_id) {
            this->diag_rx_queue = rx_queue;
            this->diag_rx_id = rx_id;
        }

    protected:
        const char* name;
        TaskHandle_t* tx_task = nullptr;
        TaskHandle_t* rx_task = nullptr;
        uint8_t tx_time_ms = 0;

        uint16_t diag_tx_id = 0;
        uint16_t diag_rx_id = 0;

        [[noreturn]]
        void tx_task_loop(void);
        [[noreturn]]
        void rx_task_loop(void);

        static void start_rx_task_loop(void *_this) {
            static_cast<EgsBaseCan*>(_this)->rx_task_loop();
        }
        static void start_tx_task_loop(void *_this) {
            static_cast<EgsBaseCan*>(_this)->tx_task_loop();
        }

        virtual void tx_frames(){};
        virtual void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) {};
        virtual void on_rx_done(uint64_t now_ts){};

        bool send_messages = true;

        QueueHandle_t* diag_rx_queue;
        twai_status_info_t can_status;
        esp_err_t can_init_status;

        inline void to_bytes(uint64_t src, uint8_t* dst) {
            for(uint8_t i = 0; i < 8; i++) {
                dst[7-i] = src & 0xFF;
                src >>= 8;
            }
        }
};

extern EgsBaseCan* egs_can_hal;

typedef uint8_t DiagCanMessage[8];

#endif