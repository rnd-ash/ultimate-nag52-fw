#ifndef CUSTOMCAN_CAN_H
#define CUSTOMCAN_CAN_H
#include "can_hal.h"
#include "shifter/shifter.h"
#include "../../lib/customcan_ecus/src/BRAKES.h"
#include "../../lib/customcan_ecus/src/WHEELS.h"
#include "../../lib/customcan_ecus/src/ENGINE.h"
#include "../../lib/customcan_ecus/src/UN52.h"
#include "../../egs52_ecus/src/EWM.h"

class CustomCan: public EgsBaseCan {
    public:
        CustomCan(const char* name, uint8_t tx_time_ms, uint32_t baud, Shifter* shifter);

        /**
         * Getters
         */

        // Get the front right wheel data
        uint16_t get_front_right_wheel(const uint32_t expire_time_ms)  override;
        // Get the front left wheel data
        uint16_t get_front_left_wheel(const uint32_t expire_time_ms) override;
        // Get the rear right wheel data
        uint16_t get_rear_right_wheel(const uint32_t expire_time_ms) override;
        // Get the rear left wheel data
        uint16_t get_rear_left_wheel(const uint32_t expire_time_ms) override;
        // // Gets the shifter position
        // ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;        
        // Gets engine type
        EngineType get_engine_type(const uint32_t expire_time_ms) override;
        // Returns true if engine is in limp mode
        bool get_engine_is_limp(const uint32_t expire_time_ms) override;
        // Returns true if pedal is kickdown 
         bool get_kickdown(const uint32_t expire_time_ms) override;
        // Returns the pedal percentage. Range 0-250
         uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        // Gets Torque information
        CanTorqueData get_torque_data(const uint32_t expire_time_ms) override;
        // Gets the flappy paddle position
         PaddlePosition get_paddle_position(const uint32_t expire_time_ms) override;
        // Gets engine coolant temperature
         int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) override;
        // Gets engine oil temperature
         int16_t get_engine_oil_temp(const uint32_t expire_time_ms) override;
         // Gets engine charge air temperature
        int16_t get_engine_iat_temp(const uint32_t expire_time_ms) override;
        // Gets engine RPM
         uint16_t get_engine_rpm(const uint32_t expire_time_ms) override;
        // Returns true if engine is cranking
        bool get_is_starting(const uint32_t expire_time_ms) override;
        bool get_profile_btn_press(const uint32_t expire_time_ms) override;
        uint16_t get_fuel_flow_rate(const uint32_t expire_time_ms) override;
        // 
        bool get_is_brake_pressed(const uint32_t expire_time_ms) override;
        TccReqState get_engine_tcc_override_request(const uint32_t expire_time_ms) override;

        /**
         * Setters
         */
        void set_clutch_status(TccClutchStatus status) override;
        // Set the actual gear of the gearbox
        void set_actual_gear(GearboxGear actual) override;
        // Set the target gear of the gearbox
        void set_target_gear(GearboxGear target) override;
        // // Sets the status bit indicating the car is safe to start
        // void set_safe_start(bool can_start) override;
        // Sets the gerabox ATF temperature. Offset by +50C
        void set_gearbox_temperature(int16_t temp) override;
        // Sets the RPM of the input shaft of the gearbox on CAN
        void set_input_shaft_speed(uint16_t rpm) override;
        // Sets 4WD activated toggle bit
        void set_is_all_wheel_drive(bool is_4wd) override;
        // Sets wheel torqu
        void set_wheel_torque(uint16_t t) override;
        // Sets shifter position message
        void set_shifter_position(ShifterPosition pos) override;
        // Sets gearbox is OK
        void set_gearbox_ok(bool is_ok) override;
        // Sets torque request toggle
        void set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) override;
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
        void set_wheel_torque_multi_factor(float ratio) override;
        void set_garage_shift_state(bool enable) override;
    protected:
        void tx_frames() override;
        void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint32_t timestamp) override;
    private:
        // CAN Frames to Tx
        bool toggle_bit = false;
        ECU_BRAKES brakes = ECU_BRAKES();
        ECU_ENGINE engine = ECU_ENGINE();
        ECU_WHEELS wheels = ECU_WHEELS();
        ECU_EWM ewm = ECU_EWM();
        
        UN52_400_CUSTOMCAN tx_400 = {};
        UN52_410_CUSTOMCAN tx_410 = {};
};

#endif // EGS51_CAN_H