
/**
 * CANBUS abstraction layer for EGS52 AND EGS53!
 */

#ifndef __ABSTRACT_CAN_H_
#define __ABSTRACT_CAN_H_

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

enum class WheelDirection {
    Forward, // Wheel going forwards
    Reverse, // Wheel going backwards
    Stationary, // Stationary (Not forward or backwards)
    SignalNotAvaliable // SNV
};

struct WheelData {
    uint16_t double_rpm; // 2x real RPM (Better accuracy)
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
    Park,
    Reverse_First,
    Reverse_Second,
    Neutral,
    First,
    Second,
    Third,
    Fourth,
    Fifth,
    Sixth,
    Seventh
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

enum class GearboxDisplayGear {
    // Shows '1'
    First,
    // Shows '2'
    Second,
    // Shows '3'
    Third,
    // Shows '4'
    Fourth,
    // Shows '5'
    Fifth,
    // Shows '6'
    Sixth,
    // Shows '7'
    Seventh,
    // Shows 'P'
    Park,
    // Shows 'R'
    Reverse,
    // Shows 'N'
    Neutral,
    // Shows 'D'
    Drive,
    // Shows 'A'
    AllWheelDrive,
    // Shows 'F' (Gearbox failure!)
    FAILURE
};

enum class GearboxMessage {
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
        };
        virtual bool begin_tasks();
        ~AbstractCan();


        /**
         * Getters
         */

        // Get the front right wheel data
        virtual WheelData get_front_right_wheel();
        // Get the front left wheel data
        virtual WheelData get_front_left_wheel();
        // Get the rear right wheel data
        virtual WheelData get_rear_right_wheel();
        // Get the rear left wheel data
        virtual WheelData get_rear_left_wheel();
        // Gets shifter position from EWM module
        virtual ShifterPosition get_shifter_position_ewm();
        // Gets engine type
        virtual EngineType get_engine_type();
        // Returns true if engine is in limp mode
        virtual bool get_engine_is_limp();
        // Returns true if pedal is kickdown 
        virtual bool get_kickdown();
        // Returns the pedal percentage. Range 0-250
        virtual uint8_t get_pedal_value();
        // Gets the current 'static' torque produced by the engine
        virtual uint16_t get_static_engine_torque();
        // Gets the maximum engine torque allowed at this moment by the engine map
        virtual uint16_t get_maximum_engine_torque();
        // Gets the minimum engine torque allowed at this moment by the engine map
        virtual uint16_t get_minimum_engine_torque();
        // Gets the flappy paddle position
        virtual PaddlePosition get_paddle_position();
        // Gets engine coolant temperature
        virtual uint16_t get_engine_coolant_temp();
        // Gets engine oil temperature
        virtual uint16_t get_engine_oil_temp();
        // Gets engine RPM
        virtual uint16_t get_engine_rpm();
        // Returns true if engine is cranking
        virtual bool get_is_starting();

        /**
         * Setters
         */

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
        // Sets the status of system error check
        virtual void set_error_check_status(SystemStatusCheck ssc);
    protected:
        TaskHandle_t* tx_task = nullptr;
        TaskHandle_t* rx_task = nullptr;
        uint8_t tx_time_ms = 0;

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
    private:
        const char* name;
};

extern AbstractCan* egs_can_hal;

#endif