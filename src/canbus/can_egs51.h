#ifndef EGS51_CAN_H
#define EGS51_CAN_H
#include "can_hal.h"
#include "../../egs51_ecus/src/GS51.h"
#include "../../egs51_ecus/src/MS51.h"
#include "../../egs51_ecus/src/ESP51.h"
#include "../../egs52_ecus/src/EWM.h"
#include "shifter/shifter.h"

class Egs51Can: public EgsBaseCan {
    public:
        Egs51Can(const char* name, uint8_t tx_time_ms, uint32_t baud);

        /**
         * Getters
         */

        // Get the front right wheel data (Double RPM is returned)
        uint16_t get_front_right_wheel(const uint32_t expire_time_ms)  override;
        // Get the front left wheel data (Double RPM is returned)
        uint16_t get_front_left_wheel(const uint32_t expire_time_ms) override;
        // Get the rear right wheel data (Double RPM is returned)
        uint16_t get_rear_right_wheel(const uint32_t expire_time_ms) override;
        // Get the rear left wheel data (Double RPM is returned)
        uint16_t get_rear_left_wheel(const uint32_t expire_time_ms) override;
        // // Gets the shifter position
        ShifterPosition internal_can_shifter_get_shifter_position(const uint32_t expire_time_ms) override;    
        // Gets engine type
        EngineType get_engine_type(const uint32_t expire_time_ms) override;
        // Returns true if engine is in limp mode
        bool get_engine_is_limp(const uint32_t expire_time_ms) override;
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
        int16_t torque_before_request=0;
        // CAN Frames to Tx
        GS_218_EGS51 gs218 = {0};
        ECU_MS51 ms51 = ECU_MS51();
        ECU_EWM ewm = ECU_EWM();        
        ECU_ESP51 esp51 = ECU_ESP51();
        uint8_t cvn_counter = 0; 
        bool freeze_torque = false;
        int16_t req_static_torque_delta = 0;
};

#endif // EGS51_CAN_H