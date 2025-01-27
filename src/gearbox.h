// Here we go, gearbox controller code! Lets go!

#ifndef GEARBOX_H
#define GEARBOX_H

#include <stdint.h>
// #include "canbus/can_hal.h"
#include "solenoids/solenoids.h"
#include "sensors.h"
#include "profiles.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common_structs.h"
#include "torque_converter.h"
#include "behaviour/driving_profiler.h"
#include "pressure_manager.h"
#include "models/input_torque.hpp"
#include "adaptation/shift_adaptation.h"
#include "models/clutch_speed.hpp"
#include "shifter/shifter.h"
//#include "runtime_sensors/runtime_sensors.h"

/* unused */
// struct PostShiftTorqueRamp {
//     bool enabled;
//     uint16_t start_nm;
//     uint16_t time_to_exit;
// };

class Gearbox {
public:
    explicit Gearbox(Shifter* shifter);
    // Diag test
    ClutchSpeeds diag_get_clutch_speeds(void);
    void set_profile(AbstractProfile* prof);
    /* unused       */
    // void inc_subprofile(void);
    esp_err_t start_controller(void);
    void inc_gear_request(void);
    void dec_gear_request(void);
    void diag_inhibit_control(void) { this->diag_stop_control = true; }
    void diag_regain_control(void) { this->diag_stop_control = false; }
    SensorData sensor_data;
    OutputData output_data;
    uint16_t get_gear_ratio(void) const {
        return this->sensor_data.gear_ratio * 100.0F;
    }
    bool shifting = false;
    PressureManager* pressure_mgr = nullptr;

    bool isShifting(void) const { return this->shifting; }
    ProfileGearChange get_curr_gear_change(void) const { return this->shift_idx; }
    TorqueConverter* tcc = nullptr;
    ShiftClutchVelocity shifting_velocity = {0,0};
    ShiftAdaptationSystem* shift_adapter = nullptr;
    SpeedSensors speed_sensors;
private:
    uint16_t redline_rpm = 4000u;
    bool is_stationary(void) const;
    ShiftReportSegment collect_report_segment(uint64_t start_time);
    void set_torque_request(TorqueRequestControlType ctrl_type, TorqueRequestBounds bounds, float amount);
    bool elapse_shift(ProfileGearChange req_lookup, AbstractProfile* profile);
    bool calcGearFromRatio(bool is_reverse);

    AbstractProfile* current_profile = nullptr;
    portMUX_TYPE profile_mutex;
    GearboxGear target_gear = GearboxGear::Park;
    GearboxGear actual_gear = GearboxGear::Park;
    GearboxGear last_fwd_gear = GearboxGear::Second;
    bool process_speed_sensors();
    [[noreturn]]
    void controller_loop(void);

    void shift_thread(void);
    bool start_second = true; // By default
    static void start_shift_thread(void *_this) {
        static_cast<Gearbox*>(_this)->shift_thread();
    }

    [[noreturn]]
    static void start_controller_internal(void *_this) {
        static_cast<Gearbox*>(_this)->controller_loop();
    }
    uint16_t temp_raw = 0;
    uint8_t pedal_last = 0;
    TaskHandle_t shift_task = nullptr;
    bool ask_upshift = false;
    bool ask_downshift = false;
    bool is_upshift = false;
    bool fwd_gear_shift = false;
    float tcc_percent = 0.F;
    uint8_t est_gear_idx = 0;
    uint16_t curr_hold_pressure = 0;
    bool show_upshift = false;
    bool show_downshift = false;
    bool flaring = false;
    int gear_disagree_count = 0;
    unsigned long last_tcc_adjust_time = 0;
    int mpc_working = 0;
    bool diag_stop_control = false;
    Shifter* shifter = nullptr;
    ShifterPosition shifter_pos = ShifterPosition::SignalNotAvailable;
    GearboxConfiguration gearboxConfig;
    ShiftCircuit last_shift_circuit;
    float diff_ratio_f;
    ProfileGearChange shift_idx = ProfileGearChange::ONE_TWO;
    bool abort_shift = false;
    bool aborting = false;
    GearboxGear restrict_target = GearboxGear::Fifth;
    GearboxGear last_motion_gear = GearboxGear::Second;
    /* unused */
    // float calc_torque_reduction_factor(ProfileGearChange change, uint16_t shift_speed_ms);
    FirstOrderAverage* pedal_average = new FirstOrderAverage(25);
    FirstOrderAverage* motor_speed_average = nullptr;

    FirstOrderAverage* engine_torque_average = nullptr;
    FirstOrderAverage* input_torque_average = nullptr;

    FirstOrderAverage* torque_req_average = nullptr;

    int req_static_torque_delta = 0;
    bool freeze_torque = false;

};

extern Gearbox* gearbox;

#endif