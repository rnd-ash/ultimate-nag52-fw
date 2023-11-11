#ifndef CAN_STRUCTS_H
#define CAN_STRUCTS_H

#include <stdint.h>

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

/// @brief Torque request control type
enum class TorqueRequestControlType: uint8_t {
    /// @brief No torque request ([TorqueRequestBounds] is ignored)
    None = 0,
    /// @brief Let the engine naturally reduce torque (Usually done via spark retarding or air reduction)
    NormalSpeed = 1,
    /// @brief As fast as possible (Usually done via fuel cut)
    FastAsPossible = 2,
    /// @brief Special case to signal to the engine that we are ramping back up to allow it to adjust ignition angle faster (EGS52)
    BackToDemandTorque = 3
};

/// @brief Torque request intervention bounds
enum class TorqueRequestBounds: uint8_t {
    /// @brief Engine makes torque less or equal to whats asked by EGS
    LessThan = 0,
    /// @brief Engine makes torque more or equal to whats asked by EGS
    MoreThan = 1,
    /// @brief Engine tries to make exactly what EGS specifies
    Exact = 2,
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

enum class TccClutchStatus: uint8_t {
    Open,
    OpenToSlipping,
    Slipping,
    SlippingToClosed,
    Closed
};

enum class TransferCaseState: uint8_t {
    Hi, // High range
    Low, // Low range
    Neither, // Neutral
    Switching, // Switching in progress
    SNA = 0xFF, // Error
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

#endif