#include "abstract_can.h"

#if EGS53_MODE == false

#ifndef EGS52_CAN_H_
#define EGS52_CAN_H_

#include "abstract_can.h"
#include <GS.h>
#include <BS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


class Egs52Can : public AbstractCanHandler {
    public:
        Egs52Can();
        ~Egs52Can();
        void start_tx_rx_loop();

        uint16_t get_engine_rpm();
        void get_rr_rpm(WheelRotation *dest);
        void get_rl_rpm(WheelRotation *dest);
        void get_fr_rpm(WheelRotation *dest);
        void get_fl_rpm(WheelRotation *dest);
        int16_t get_steering_angle();
        int16_t get_ambient_temp();
        int16_t get_engine_temp();
        bool is_profile_toggle_pressed();

        Gear get_abs_target_lower_gear();
        Gear get_abs_target_upper_gear();
        bool get_abs_request_downshift();
        bool get_abs_request_gear_forced();

        uint16_t get_engine_static_torque();
        uint16_t get_engine_max_torque_dyno();
        uint16_t get_engine_max_torque();
        uint16_t get_engine_min_torque();

        // Setters
        void set_is_safe_start(bool can_start);
        void set_atf_temp(uint16_t temp);
        void set_drive_profile(DriveProfileDisplay p);
        void set_display_message(DisplayMessage m);
        void set_target_gear(Gear g);
        void set_actual_gear(Gear g);
        void set_turbine_rpm(uint16_t rpm);
        void set_torque_loss_nm(uint16_t loss);
        void set_display_speed_step(SpeedStep disp);
        void set_status_error_check(ErrorCheck e);
        void set_shifter_possition(ShifterPosition g);

    private:
        static void __start_thread_tx(void *_this);
        static void __start_thread_rx(void *_this);
        void tx_loop();
        void rx_loop();
        TaskHandle_t* task_handler_tx = nullptr;
        TaskHandle_t* task_handler_rx = nullptr;
        DriveProfileDisplay temp = DriveProfileDisplay::SNV;

        BS_ECU bsECU; // ABS / BAS / ESP module
};

extern Egs52Can* egs52_can_handler;

#endif // EGS52_CAN_H_
#endif // EGS53_MODE