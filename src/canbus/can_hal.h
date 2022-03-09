
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

enum class WheelDirection {
    Forward, // Wheel going forwards
    Reverse, // Wheel going backwards
    Stationary, // Stationary (Not forward or backwards)
    SignalNotAvaliable // SNV
};

struct WheelData {
    int double_rpm; // 2x real RPM (Better accuracy)
    WheelDirection current_dir; // Wheel direction
};

struct DiagIsoTpInfo {
    uint32_t tx_canid;
    uint32_t rx_canid;
    uint8_t bs;
    uint8_t st_min;
};

enum class SystemStatusCheck {
    // Waiting for check to complete
    Waiting,
    // No errors. Gearbox is OK
    OK,
    // Errors found
    Error
};

enum class EngineType {
    Diesel,
    Petrol,
    Unknown
};

enum class TorqueRequest {
    Maximum,
    Minimum,
    None
};

enum class GearboxGear {
    First = 1,
    Second = 2,
    Third = 3,
    Fourth = 4,
    Fifth = 5,
    Park = 8,
    Neutral = 9,
    Reverse_First = 10,
    Reverse_Second = 11,
    SignalNotAvaliable = 12,
};

enum class PaddlePosition {
    None,
    Plus,
    Minus,
    PlusAndMinus
};

enum class ClutchStatus {
    Open,
    Slipping,
    Closed
};

enum class GearboxProfile {
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
    // Reserved for 'no profile' (_)
    Underscore
};

enum class ShifterPosition {
    P,
    P_R,
    R,
    R_N,
    N,
    N_D,
    D,
    PLUS,
    MINUS,
    SignalNotAvaliable // SNV
};

enum class SolenoidName {
    Y3,
    Y4,
    Y5,
    SPC,
    MPC,
    TCC
};

enum class GearboxDisplayGear {
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
    SNA,
};

enum class GearboxMessage {
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

class AbstractCan {
    public:
        explicit AbstractCan(const char* name, uint8_t tx_time_ms) {
            this->name = name;
            this->tx_time_ms = tx_time_ms;
            this->tx_task = nullptr;
            this->rx_task = nullptr;
            this->diag_tx_queue = nullptr;
            this->diag_rx_queue = nullptr;
        };
        virtual bool begin_tasks();
        ~AbstractCan();


        /**
         * Getters
         */

        // Get the front right wheel data
        virtual WheelData get_front_right_wheel(uint64_t now, uint64_t expire_time_ms);
        // Get the front left wheel data
        virtual WheelData get_front_left_wheel(uint64_t now, uint64_t expire_time_ms);
        // Get the rear right wheel data
        virtual WheelData get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms);
        // Get the rear left wheel data
        virtual WheelData get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms);
        // Gets shifter position from EWM module
        virtual ShifterPosition get_shifter_position_ewm(uint64_t now, uint64_t expire_time_ms);
        // Gets engine type
        virtual EngineType get_engine_type(uint64_t now, uint64_t expire_time_ms);
        // Returns true if engine is in limp mode
        virtual bool get_engine_is_limp(uint64_t now, uint64_t expire_time_ms);
        // Returns true if pedal is kickdown 
        virtual bool get_kickdown(uint64_t now, uint64_t expire_time_ms);
        // Returns the pedal percentage. Range 0-250
        virtual uint8_t get_pedal_value(uint64_t now, uint64_t expire_time_ms);
        // Gets the current 'static' torque produced by the engine
        virtual int get_static_engine_torque(uint64_t now, uint64_t expire_time_ms);
        // Gets the maximum engine torque allowed at this moment by the engine map
        virtual int get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms);
        // Gets the minimum engine torque allowed at this moment by the engine map
        virtual int get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms);
        // Gets the flappy paddle position
        virtual PaddlePosition get_paddle_position(uint64_t now, uint64_t expire_time_ms);
        // Gets engine coolant temperature
        virtual int16_t get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms);
        // Gets engine oil temperature
        virtual int16_t get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms);
        // Gets engine RPM
        virtual uint16_t get_engine_rpm(uint64_t now, uint64_t expire_time_ms);
        // Returns true if engine is cranking
        virtual bool get_is_starting(uint64_t now, uint64_t expire_time_ms);
        virtual bool get_profile_btn_press(uint64_t now, uint64_t expire_time_ms);
        virtual bool get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms);

        /**
         * Setters
         */

        virtual void set_race_start(bool race_start);
        // Set solenoid PMW
        virtual void set_solenoid_pwm(uint8_t duty, SolenoidName s);
        virtual void set_last_shift_time(uint16_t time_ms);
        // Set the gearbox clutch position on CAN
        virtual void set_clutch_status(ClutchStatus status);
        // Set the actual gear of the gearbox
        virtual void set_actual_gear(GearboxGear actual);
        // Set the target gear of the gearbox
        virtual void set_target_gear(GearboxGear target);
        // Sets the status bit indicating the car is safe to start
        virtual void set_safe_start(bool can_start);
        // Sets the gerabox ATF temperature. Offset by +50C
        virtual void set_gearbox_temperature(uint16_t temp);
        // Sets the RPM of the input shaft of the gearbox on CAN
        virtual void set_input_shaft_speed(uint16_t rpm);
        // Sets 4WD activated toggle bit
        virtual void set_is_all_wheel_drive(bool is_4wd);
        // Sets wheel torque
        virtual void set_wheel_torque(uint16_t t);
        // Sets shifter position message
        virtual void set_shifter_position(ShifterPosition pos);
        // Sets gearbox is OK
        virtual void set_gearbox_ok(bool is_ok);
        // Sets torque request toggle
        virtual void set_torque_request(TorqueRequest request);
        // Sets requested engine torque
        virtual void set_requested_torque(uint16_t torque_nm);
        // Sets torque loss of torque converter
        virtual void set_turbine_torque_loss(uint16_t loss_nm);
        // Sets torque multiplier factor from Engine all the way to wheels 
        virtual void set_wheel_torque_multi_factor(float ratio);
        // Sets the status of system error check
        virtual void set_error_check_status(SystemStatusCheck ssc);
        // Sets display profile
        virtual void set_display_gear(GearboxDisplayGear g, bool manual_mode);
        // Sets drive profile
        virtual void set_drive_profile(GearboxProfile p);
        // Sets display message
        virtual void set_display_msg(GearboxMessage msg);

        // For diagnostic passive mode
        void enable_normal_msg_transmission() {
            this->send_messages = true;
        }

        // For diagnostic passive mode
        void disable_normal_msg_transmission() {
            this->send_messages = false;
        }

        // For diagnostics
        void register_diag_queue(QueueHandle_t* rx_queue, uint16_t rx_id, QueueHandle_t* tx_queue, uint16_t tx_id) {
            this->diag_rx_queue = rx_queue;
            this->diag_tx_queue = tx_queue;
            this->diag_rx_id = rx_id;
            this->diag_tx_id = tx_id;
        }

    protected:
        TaskHandle_t* tx_task = nullptr;
        TaskHandle_t* rx_task = nullptr;
        uint8_t tx_time_ms = 0;

        uint16_t diag_tx_id = 0;
        uint16_t diag_rx_id = 0;

        [[noreturn]]
        virtual void tx_task_loop();
        [[noreturn]]
        virtual void rx_task_loop();

        static void start_rx_task_loop(void *_this) {
            static_cast<AbstractCan*>(_this)->rx_task_loop();
        }
        static void start_tx_task_loop(void *_this) {
            static_cast<AbstractCan*>(_this)->tx_task_loop();
        }
        bool send_messages = true;

        QueueHandle_t* diag_rx_queue;
        QueueHandle_t* diag_tx_queue;
    private:
        const char* name;
};

extern AbstractCan* egs_can_hal;

typedef uint8_t DiagCanMessage[8];

#endif