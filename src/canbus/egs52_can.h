#if EGS53_MODE == false

#ifndef EGS52_CAN_H_
#define EGS52_CAN_H_

#include "abstract_can.h"
#include <GS.h>
#include <BS.h>
#include <EWM.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iso_tp.h"


class Egs52Can : public AbstractCanHandler {
    public:
        Egs52Can();
        ~Egs52Can();
        void start_tx_rx_loop();

        // Getters
        uint16_t get_engine_rpm();
        void get_rr_rpm(WheelRotation *dest) override;
        void get_rl_rpm(WheelRotation *dest) override;
        void get_fr_rpm(WheelRotation *dest) override;
        void get_fl_rpm(WheelRotation *dest) override;
        int16_t get_steering_angle() override;
        int16_t get_ambient_temp() override;
        int16_t get_engine_temp() override;
        bool is_profile_toggle_pressed() override;

        Gear get_abs_target_lower_gear() override;
        Gear get_abs_target_upper_gear() override;
        bool get_abs_request_downshift() override;
        bool get_abs_request_gear_forced() override;

        uint16_t get_engine_static_torque() override;
        uint16_t get_engine_max_torque_dyno() override;
        uint16_t get_engine_max_torque() override;
        uint16_t get_engine_min_torque() override;
        ShifterPosition get_shifter_position() override;

        // Setters
        void set_is_safe_start(bool can_start) override;
        void set_atf_temp(uint16_t temp) override;
        void set_drive_profile(DriveProfileDisplay p) override;
        void set_display_message(DisplayMessage m) override;
        void set_target_gear(Gear g) override;
        void set_actual_gear(Gear g) override;
        void set_turbine_rpm(uint16_t rpm) override;
        void set_torque_loss_nm(uint16_t loss) override;
        void set_display_speed_step(SpeedStep disp) override;
        void set_status_error_check(ErrorCheck e) override;
        void set_shifter_possition(ShifterPosition g) override;

    private:
        static void __start_thread_tx(void *_this);
        static void __start_thread_rx(void *_this);
        void tx_loop();
        void rx_loop();
        TaskHandle_t* task_handler_tx = nullptr;
        TaskHandle_t* task_handler_rx = nullptr;
        DriveProfileDisplay temp = DriveProfileDisplay::SNV;

        BS_ECU bsECU; // ABS / BAS / ESP module
        EWM_ECU ewmECU; // Shift lever
        IsoTpServer* diag_endpoint;
};

extern Egs52Can* egs52_can_handler;

#endif // EGS52_CAN_H_
#endif // EGS53_MODE