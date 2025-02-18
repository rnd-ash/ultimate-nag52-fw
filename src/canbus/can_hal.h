
/**
 * CANBUS abstraction layer for EGS52 AND EGS53!
 */

#ifndef ABSTRACT_CAN_H
#define ABSTRACT_CAN_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <freertos/queue.h>
#include <string.h>
#include "driver/twai.h"
#include "clock.hpp"

#include "../../slave_ecus/src/EGS_SLAVE.h"
#include "../../slave_ecus/src/TESTER.h"
#include "can_defines.h"

#include "../shifter/shifter.h"
#include "../shifter/programselector/programselector.hpp"
#include "../profiles.h"

class EgsBaseCan {
    public:
        EgsBaseCan(const char* name, uint8_t tx_time_ms, uint32_t baud, Shifter* shifter);

        bool bus_ok() const;

        ~EgsBaseCan();        
        bool begin_task();
        esp_err_t init_state() const;

        /**
         * Getters
         */

        
        /**
         * @brief OPTIONAL DATA - Returns the front right wheel speed (In half RPM increments)
         * @param expire_time_ms data expiration period
         * @return front right wheel data
         */
        virtual uint16_t get_front_right_wheel(const uint32_t expire_time_ms) {
            return UINT16_MAX;
        }
        
        /**
         * @brief OPTIONAL DATA - Returns the front left wheel speed (In half RPM increments)
         * @param expire_time_ms data expiration period
         * @return front left wheel data
         */
        virtual uint16_t get_front_left_wheel(const uint32_t expire_time_ms) {
            return UINT16_MAX;
        }
        
        /**
         * @brief MANDITORY DATA (If no dedicated output shaft sensor) - Returns the rear right wheel speed (In half RPM increments)
         * @param expire_time_ms data expiration period
         * @return rear right wheel data
         */
        virtual uint16_t get_rear_right_wheel(const uint32_t expire_time_ms) {
            return UINT16_MAX;
        }
        
        /**
         * @brief MANDITORY DATA (If no dedicated output shaft sensor) - Returns the left right wheel speed (In half RPM increments)
         * @param expire_time_ms data expiration period
         * @return rear right left data
         */
        virtual uint16_t get_rear_left_wheel(const uint32_t expire_time_ms) {
            return UINT16_MAX;
        }
        

        ShifterPosition get_shifter_position(const uint32_t expire_time_ms) {
            if (shifter) {
                return shifter->get_shifter_position(expire_time_ms);
            } else {
                return ShifterPosition::SignalNotAvailable;
            }
        }

        /**
         * @brief Only Call from Shifter!
         * @param expire_time_ms 
         * @return 
         */
        virtual ShifterPosition internal_can_shifter_get_shifter_position(const uint32_t expire_time_ms) {
            return ShifterPosition::SignalNotAvailable;
        }

        /**
         * @brief TBA - Returns the type of engine in the vehicle (Detected over CAN)
         * @param expire_time_ms data expiration period
         * @return Engine type (Diesel or Petrol)
         */
        virtual EngineType get_engine_type(const uint32_t expire_time_ms) {
            return EngineType::Unknown;
        }
        
        /**
         * @brief TBA - Returns true if the engine is in limp mode
         * @param expire_time_ms data expiration period
         * @return If the engine is in limp mode
         */
        virtual bool get_engine_is_limp(const uint32_t expire_time_ms) {
            return false;
        }
        
        /**
         * @brief OPTIONAL DATA (For kickdown only) - Returns if the kickdown switch is actuated
         * @param expire_time_ms data expiration period
         * @return Returns true if the kickdown switch is being pressed, false if released
         */
        virtual bool get_kickdown(const uint32_t expire_time_ms) {
            return 0;
        }
        
        /**
         * @brief MANDITORY DATA - Returns the pedal position, on a scale of 0-250
         * @param expire_time_ms data expiration period
         * @return Pedal position (0 = 0%, 250 = 100%)
         */
        virtual uint8_t get_pedal_value(const uint32_t expire_time_ms) {
            return 0;
        }
        
        /**
         * @brief MANDITORY DATA - Returns the current output (Static) engine torque
         * This function may return a negative number to indicate the engine is acting as a drag source 
         * when coasting
         * 
         * @param expire_time_ms data expiration period
         * @return Static engine torque in Nm (In Nm)
         */
        virtual int get_static_engine_torque(const uint32_t expire_time_ms) {
            return 0;
        }
        
        /**
         * @brief MANDITORY DATA - Returns the amount of torque the engine has been asked to make
         * by the drivers pedal position
         * 
         * @param expire_time_ms data expiration period
         * @return Driver requested engine output torque (In Nm)
         */
        virtual int get_driver_engine_torque(const uint32_t expire_time_ms) {
            return 0;
        }
        
        /**
         * @brief MANDITORY DATA - Returns the maximum engine output torque based on RPM
         * @param expire_time_ms data expiration period
         * @return Engine maximum possible production torque (In Nm)
         */
        virtual int get_maximum_engine_torque(const uint32_t expire_time_ms) {
            return 0;
        }
        
        /**
         * @brief MANDITORY DATA - Returns the minimum engine output torque based on RPM
         * @param expire_time_ms data expiration period
         * @return Engine minimum possible production torque (In Nm)
         */
        virtual int get_minimum_engine_torque(const uint32_t expire_time_ms) {
            return 0;
        }
        
        /**
         * @brief OPTIONAL DATA - Returns the drag torque imposed by the AC compressor on the engine
         * @param expire_time_ms data expiration period
         * @return AC torque loss (In Nm)
         */
        virtual uint8_t get_ac_torque_loss(const uint32_t expire_time_ms) {
            return UINT8_MAX;
        }

        /**
         * @brief OPTIONAL DATA - Returns the position of shift paddles (If fitted)
         * @param expire_time_ms data expiration period
         * @return Engine maximum possible production torque
         */
        virtual PaddlePosition get_paddle_position(const uint32_t expire_time_ms) {
            return PaddlePosition::SNV;
        }
        
        /**
         * @brief OPTIONAL DATA - Returns the engines coolant temperature in Celcius
         * @param expire_time_ms data expiration period
         * @return Engine coolant temperature (In Grad. C)
         */
        virtual int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) {
            return INT16_MAX;
        }
        
        /**
         * @brief OPTIONAL DATA - Returns the engines oil temperature in Celcius
         * @param expire_time_ms data expiration period
         * @return Engine oil temperature (In Grad. C)
         */
        virtual int16_t get_engine_oil_temp(const uint32_t expire_time_ms) {
            return INT16_MAX;
        }
        
        /**
         * @brief OPTIONAL DATA - Returns the engines intake air temperature in Celcius
         * @param expire_time_ms data expiration period
         * @return Engine intake air temperature (In Grad. C)
         */
        virtual int16_t get_engine_iat_temp(const uint32_t expire_time_ms) {
            return INT16_MAX;
        }
       
        /**
         * @brief MANDITORY DATA - Returns the engines speed in RPM
         * @param expire_time_ms data expiration period
         * @return Engine speed (In RPM)
         */
        virtual uint16_t get_engine_rpm(const uint32_t expire_time_ms) {
            return UINT16_MAX;
        }
        
        /**
         * @brief TBA - Returns true if the engine is cranking to start up
         * @param expire_time_ms data expiration period
         * @return True if engine is cranking, False is engine is off or running
         */
        virtual bool get_is_starting(const uint32_t expire_time_ms) {
            return false;
        }

        /**
         * @brief OPTIONAL DATA - Returns true if the shifter profile button is pressed
         * This is intended for only the handling of the EWM CAN based shifters.
         * 
         * @param expire_time_ms data expiration period
         * @return True if the profile button is in the depressed position, or False if it is released
         */
        virtual bool get_profile_btn_press(const uint32_t expire_time_ms) {
            return false;
        }

        /**
         * @brief OPTIONAL DATA - Returns true if the shifter profile switch is in the top most position
         * This is intended for only the handling of the EWM CAN based shifters.
         * 
         * @param expire_time_ms data expiration period
         * @return True if the profile switch is in the top position
         */
        virtual ProfileSwitchPos get_profile_switch_pos(const uint32_t expire_time_ms) {
            return ProfileSwitchPos::SNV;
        }

        /**
         * @brief MANDITORY DATA - Returns if the brake pedal is being pressed
         * 
         * @param expire_time_ms data expiration period
         * @return True if the brake pedal is pressed, false if the brake pedal is released
         */
        virtual bool get_is_brake_pressed(const uint32_t expire_time_ms) {
            return false;
        }

        /**
         * @brief OPTIONAL DATA - Returns fuel consumption rate of the engine
         * 
         * @param expire_time_ms data expiration period
         * @return Fuel consumption of the engine (In μL/250ms)
         */
        virtual uint16_t get_fuel_flow_rate(const uint32_t expire_time_ms) {
            return 0;
        }
        virtual TransferCaseState get_transfer_case_state(const uint32_t expire_time_ms) {
            return TransferCaseState::SNA;
        }
        virtual bool engine_ack_torque_request(const uint32_t expire_time_ms) {
            return false;
        }   

        // Checks if ESP torque intervention is active (AKA Stability assist)
        virtual bool esp_torque_intervention_active(const uint32_t expire_time_ms) {
            return false;
        }

        // Checks if cruise control is active
        virtual bool is_cruise_control_active(const uint32_t expire_time_ms) {
            return false;
        }

        // Gets the torque demand from the cruise control system
        virtual int cruise_control_torque_demand(const uint32_t expire_time_ms) {
            return INT_MAX;
        }

        // Gets the torque demand from the ESP system
        virtual int esp_torque_demand(const uint32_t expire_time_ms) {
            return INT_MAX;
        }

        virtual TccReqState get_engine_tcc_override_request(const uint32_t expire_time_ms) {
            return TccReqState::None;
        }
        
        /**
         * Setters
         */
        // Set the gearbox clutch position on CAN
        virtual void set_clutch_status(TccClutchStatus status){};
        // Set the actual gear of the gearbox
        virtual void set_actual_gear(GearboxGear actual){};
        // Set the target gear of the gearbox
        virtual void set_target_gear(GearboxGear target){};
        // Sets the status bit indicating the car is safe to start
        virtual void set_safe_start(bool can_start){};
        // Sets the gerabox ATF temperature. Offset by +50C
        virtual void set_gearbox_temperature(int16_t temp){};
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
        virtual void set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm){};
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

        virtual void set_fake_engine_rpm(uint16_t rpm){};
        // Tells the engine if we are shifting from P->R or N->D.
        // This is needed so the engine limits itself to 1K RPM, in order
        // to prevent any damage to the box!
        virtual void set_garage_shift_state(bool enable){};
        
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

        SOLENOID_CONTROL_EGS_SLAVE get_tester_req() { // Never expires
            SOLENOID_CONTROL_EGS_SLAVE dest = {0};
            this->egs_slave_mode_tester.get_SOLENOID_CONTROL(GET_CLOCK_TIME(), UINT32_MAX, &dest);
            return dest;
        }

        void set_slave_mode_reports(
            SOLENOID_REPORT_EGS_SLAVE sol_rpt,
            SENSOR_REPORT_EGS_SLAVE sensor_rpt,
            UN52_REPORT_EGS_SLAVE un52_rpt
        ) {
            this->solenoid_slave_resp = sol_rpt;
            this->sensors_slave_resp = sensor_rpt;
            this->un52_slave_resp = un52_rpt;
        }

        Shifter* shifter;

    protected:
        const char* name;
        TaskHandle_t task = nullptr;
        uint8_t tx_time_ms = 0;
        uint32_t last_tx_time = 0;
        uint16_t diag_tx_id = 0;
        uint16_t diag_rx_id = 0;

        [[noreturn]]
        void task_loop(void);

        static void start_task_loop(void *_this) {
            static_cast<EgsBaseCan*>(_this)->task_loop();
        }

        virtual void tx_frames(){};
        virtual void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) {};
        virtual void on_rx_done(const uint32_t now_ts);

        bool send_messages = true;

        QueueHandle_t* diag_rx_queue;
        twai_status_info_t can_status;
        esp_err_t can_init_status;
        twai_message_t tx;
        inline void to_bytes(uint64_t src, uint8_t* dst) {
            for(uint8_t i = 0; i < 8; i++) {
                dst[7-i] = src & 0xFF;
                src >>= 8;
            }
        }

        ECU_TESTER egs_slave_mode_tester;
        SOLENOID_REPORT_EGS_SLAVE solenoid_slave_resp;
        SENSOR_REPORT_EGS_SLAVE sensors_slave_resp;
        UN52_REPORT_EGS_SLAVE un52_slave_resp;
        uint64_t bus_reset_time = 0;
        uint8_t bus_reset_count = 0;
};

extern EgsBaseCan* egs_can_hal;

// typedef uint8_t DiagCanMessage[8];
struct DiagCanMessage {
    uint8_t data[8];
};

#endif