#ifndef __EGS53_CAN_H_
#define __EGS53_CAN_H_

#include "can_hal.h"
#include "../../egs53_ecus/src/TCM.h"
#include "../../egs53_ecus/src/TSLM.h"
#include "../../egs53_ecus/src/FSCM.h"
#include "../../egs53_ecus/src/ECM.h"
#include "../../egs53_ecus/src/ANY_ECU.h"

class Egs53Can: public EgsBaseCan {
    public:
        explicit Egs53Can(const char* name, uint8_t tx_time_ms, uint32_t baud);

        /**
         * Getters
         */

        // Get the front right wheel data
        WheelData get_front_right_wheel(const uint32_t expire_time_ms)  override;
        // Get the front left wheel data
        WheelData get_front_left_wheel(const uint32_t expire_time_ms) override;
        // Get the rear right wheel data
        WheelData get_rear_right_wheel(const uint32_t expire_time_ms) override;
        // Get the rear left wheel data
        WheelData get_rear_left_wheel(const uint32_t expire_time_ms) override;
        // Gets shifter position from EWM module
        ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
        // Gets engine type
        EngineType get_engine_type(const uint32_t expire_time_ms) override;
        // Returns true if engine is in limp mode
        bool get_engine_is_limp(const uint32_t expire_time_ms) override;
        // Returns true if pedal is kickdown 
         bool get_kickdown(const uint32_t expire_time_ms) override;
        // Returns the pedal percentage. Range 0-250
         uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        // Gets the current 'static' torque produced by the engine
         int get_static_engine_torque(const uint32_t expire_time_ms) override;
        // Gets the maximum engine torque allowed at this moment by the engine map
         int get_maximum_engine_torque(const uint32_t expire_time_ms) override;
        // Gets the minimum engine torque allowed at this moment by the engine map
         int get_minimum_engine_torque(const uint32_t expire_time_ms) override;
         int get_driver_engine_torque(const uint32_t expire_time_ms) override;
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
        // 
        bool get_profile_btn_press(const uint32_t expire_time_ms) override;
        bool get_is_brake_pressed(const uint32_t expire_time_ms) override;
        bool engine_ack_torque_request(const uint32_t expire_time_ms) override;
        bool esp_torque_intervention_active(const uint32_t expire_time_ms) override;
        bool is_cruise_control_active(const uint32_t expire_time_ms) override;
        int cruise_control_torque_demand(const uint32_t expire_time_ms) override;
        int esp_torque_demand(const uint32_t expire_time_ms) override;
        /**
         * Setters
         */

        // Set the gearbox clutch position on CAN
        void set_clutch_status(TccClutchStatus status) override;
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
        void set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) override;
        // Sets the status of system error check
        void set_error_check_status(SystemStatusCheck ssc) override;
        // Sets torque loss of torque converter
        void set_turbine_torque_loss(uint16_t loss_nm) override;
        // Sets display gear
        void set_display_gear(GearboxDisplayGear g, bool manual_mode) override;
        // Sets drive profile
        void set_drive_profile(GearboxProfile p) override;
        // Sets display message
        void set_display_msg(GearboxMessage msg) override;
        void set_wheel_torque_multi_factor(float ratio) override;
    protected:
        void tx_frames() override;
        void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) override;
    private:
        // CAN Frames to Tx
        TCM_A1_EGS53 tcm_a1 = {0};
        TCM_A2_EGS53 tcm_a2 = {0};
        ENG_RQ1_TCM_EGS53 eng_rq1_tcm = {0};
        ENG_RQ2_TCM_EGS53 eng_rq2_tcm = {0};
        ENG_RQ3_TCM_EGS53 eng_rq3_tcm = {0};
        SBW_RS_TCM_EGS53 sbw_rs_tcm = {0};
        TCM_DISP_RQ_EGS53 tcm_disp_rq = {0};
        NM_TCM_EGS53 nm_tcm = {0};
        WHL_STAT2_EGS53 eng_wheel_stat_1 = {0};
        ECU_ECM ecm_ecu = ECU_ECM();
        ECU_FSCM fscm_ecu = ECU_FSCM();
        ECU_TSLM tslm_ecu = ECU_TSLM();

        uint8_t counter = 0;
        uint8_t cvn_counter = 0;

};
#endif