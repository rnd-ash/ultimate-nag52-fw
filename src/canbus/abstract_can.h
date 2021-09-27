//
// Created by ashcon on 9/13/21.
//


#ifndef ULTIMATE_NAG52_FW_ABSTRACT_CAN_H
#define ULTIMATE_NAG52_FW_ABSTRACT_CAN_H

#include <config.h>
#include <cstdint>

enum class WheelDirection {
    PASSIVE = 0,
    FORWARD = 1,
    REVERSE = 2,
    SNV = 3
};

struct WheelRotation {
    uint16_t rpm;
    WheelDirection dir;
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
    SNV
};

enum class Paddle {
    None,
    Plus,
    Minus,
};

enum class DriveProfileDisplay {
    Agility,
    Standard,
    Comfort,
    Manual,
    Winter,
    FAILURE,
    SNV
};

enum class DisplayMessage {
    ActuateParkingBrake_WarningTone,
    ShiftLeverToN,
    ShiftLeverToNToStart,
    ApplyBrake,
    RequestGearAgain,
    Upshift,
    Downshift,
    VisitWorkshop,
    None
};

enum class SpeedStep {
    BLANK,
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX,
    SEVEN,
    A,
    D,
    F,
    N,
    P,
    R,
    SNV
};

enum class Gear {
    R2,
    R1,
    N,
    P,
    D1,
    D2,
    D3,
    D4,
    D5,
    D6,
    D7,
    SNV,
    CANCEL,
    PASSIVE
};

enum class ErrorCheck {
    WAIT,
    OK,
    ERROR
};

class AbstractCanHandler {
public:
    // Getters
    virtual uint16_t get_engine_rpm();
    virtual void get_rr_rpm(WheelRotation *dest);
    virtual void get_rl_rpm(WheelRotation *dest);
    virtual void get_fr_rpm(WheelRotation *dest);
    virtual void get_fl_rpm(WheelRotation *dest);
    virtual int16_t get_steering_angle();
    virtual int16_t get_ambient_temp();
    virtual int16_t get_engine_temp();
    virtual bool is_profile_toggle_pressed();
    virtual ShifterPosition get_shifter_position();

    virtual Gear get_abs_target_lower_gear();
    virtual Gear get_abs_target_upper_gear();
    virtual bool get_abs_request_downshift();
    virtual bool get_abs_request_gear_forced();

    virtual uint16_t get_engine_static_torque();
    virtual uint16_t get_engine_max_torque_dyno();
    virtual uint16_t get_engine_max_torque();
    virtual uint16_t get_engine_min_torque();

    // Setters
    virtual void set_is_safe_start(bool can_start);
    virtual void set_atf_temp(uint16_t temp);
    virtual void set_drive_profile(DriveProfileDisplay p);
    virtual void set_display_message(DisplayMessage m);
    virtual void set_target_gear(Gear g);
    virtual void set_actual_gear(Gear g);
    virtual void set_turbine_rpm(uint16_t rpm);
    virtual void set_torque_loss_nm(uint16_t loss);
    virtual void set_display_speed_step(SpeedStep disp);
    virtual void set_status_error_check(ErrorCheck e);
    virtual void set_shifter_possition(ShifterPosition g);
    
    uint8_t can_tx_ms;
};


#endif //ULTIMATE_NAG52_FW_ABSTRACT_CAN_H
