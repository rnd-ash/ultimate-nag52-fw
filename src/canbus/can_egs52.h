#ifndef __EGS52_CAN_H_
#define __EGS52_CAN_H_
#include <gearbox_config.h>

#ifdef EGS52_MODE
#include "can_hal.h"
#include "ANY_ECU.h"
#include "ESP_SBC.h"
#include "EWM.h"
#include "GS.h"
#include "MS.h"

class Egs52Can: public AbstractCan {
    public:
        explicit Egs52Can(const char* name, uint8_t tx_time_ms);
        bool begin_tasks() override;
        ~Egs52Can();

        /**
         * Getters
         */

        // Get the front right wheel data
        WheelData get_front_right_wheel(uint64_t now, uint64_t expire_time_ms)  override;
        // Get the front left wheel data
        WheelData get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) override;
        // Get the rear right wheel data
        WheelData get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) override;
        // Get the rear left wheel data
        WheelData get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) override;
        // Gets shifter position from EWM module
        ShifterPosition get_shifter_position_ewm(uint64_t now, uint64_t expire_time_ms) override;
        // Gets engine type
        EngineType get_engine_type(uint64_t now, uint64_t expire_time_ms) override;
        // Returns true if engine is in limp mode
        bool get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) override;
        // Returns true if pedal is kickdown 
         bool get_kickdown(uint64_t now, uint64_t expire_time_ms) override;
        // Returns the pedal percentage. Range 0-250
         uint8_t get_pedal_value(uint64_t now, uint64_t expire_time_ms) override;
        // Gets the current 'static' torque produced by the engine
         int get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) override;
        // Gets the maximum engine torque allowed at this moment by the engine map
         int get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) override;
        // Gets the minimum engine torque allowed at this moment by the engine map
         int get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) override;
        // Gets the flappy paddle position
         PaddlePosition get_paddle_position(uint64_t now, uint64_t expire_time_ms) override;
        // Gets engine coolant temperature
         int16_t get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) override;
        // Gets engine oil temperature
         int16_t get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) override;
        // Gets engine RPM
         uint16_t get_engine_rpm(uint64_t now, uint64_t expire_time_ms) override;
        // Returns true if engine is cranking
         bool get_is_starting(uint64_t now, uint64_t expire_time_ms) override;
         bool get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) override;
        // 
         bool get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) override;

        /**
         * Setters
         */

        void set_last_shift_time(uint16_t time_ms) override;
        void set_race_start(bool race_start) override;
        void set_clutch_status(ClutchStatus status) override;
        // Set the actual gear of the gearbox
        void set_actual_gear(GearboxGear actual) override;
        // Set the target gear of the gearbox
        void set_target_gear(GearboxGear target) override;
        // Sets the status bit indicating the car is safe to start
        void set_safe_start(bool can_start) override;
        // Sets the gerabox ATF temperature. Offset by +50C
        void set_gearbox_temperature(uint16_t temp) override;
        // Sets the RPM of the input shaft of the gearbox on CAN
        void set_input_shaft_speed(uint16_t rpm) override;
        // Sets 4WD activated toggle bit
        void set_is_all_wheel_drive(bool is_4wd) override;
        // Sets wheel torque
        void set_wheel_torque(uint16_t t) override;
        // Sets shifter position message
        void set_shifter_position(ShifterPosition pos) override;
        // Sets gearbox is OK
        void set_gearbox_ok(bool is_ok) override;
        // Sets torque request toggle
        void set_torque_request(TorqueRequest request) override;
        // Sets requested engine torque
        void set_requested_torque(uint16_t torque_nm) override;
        // Sets the status of system error check
        void set_error_check_status(SystemStatusCheck ssc) override;
        // Sets torque loss of torque converter
        void set_turbine_torque_loss(uint16_t loss_nm) override;
        // Sets display profile
        void set_display_gear(GearboxDisplayGear g, bool manual_mode) override;
        // Sets drive profile
        void set_drive_profile(GearboxProfile p) override;
        // Sets display message
        void set_display_msg(GearboxMessage msg) override;
        void set_solenoid_pwm(uint8_t duty, SolenoidName s) override;
    protected:
        [[noreturn]]
        void tx_task_loop() override;
        [[noreturn]]
        void rx_task_loop() override;
    private:
        GearboxProfile curr_profile_bit = GearboxProfile::Underscore;
        GearboxMessage curr_message = GearboxMessage::None;
        // CAN Frames to Tx
        GS_218 gs218 = {0};
        GS_418 gs418 = {0};
        GS_338 gs338 = {0};
        GS_558_CUSTOM gs558 = {0};
        // ECU Data to Rx to
        ECU_ESP_SBC esp_ecu = ECU_ESP_SBC();
        ECU_ANY_ECU misc_ecu = ECU_ANY_ECU();
        ECU_EWM ewm_ecu = ECU_EWM();
        ECU_MS ecu_ms = ECU_MS();
        bool can_init_ok = false;
};

#endif // EGS53_MODE

#endif // EGS52_CAN_H_