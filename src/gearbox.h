// Here we go, gearbox controller code! Lets go!

#ifndef GEARBOX_H
#define GEARBOX_H

#include <stdint.h>
#include "canbus/can_hal.h"
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
#include "inputcomponents/brakepedal.hpp"
#include "inputcomponents/kickdownswitch.hpp"
//#include "runtime_sensors/runtime_sensors.h"

struct PostShiftTorqueRamp {
    bool enabled;
    uint16_t start_nm;
    uint16_t time_to_exit;
};

class Gearbox {
public:
    explicit Gearbox(Shifter* shifter);
    // Diag test
    ClutchSpeeds diag_get_clutch_speeds();
    void set_profile(AbstractProfile* prof);
    esp_err_t start_controller(void);
    void inc_gear_request(void);
    void dec_gear_request(void);
    void diag_inhibit_control(void) { this->diag_stop_control = true; }
    void diag_regain_control(void) { this->diag_stop_control = false; }
    bool get_is_start_safe(void) {return this->is_start_safe; }
    SensorData sensor_data;
    OutputData output_data;
    uint16_t get_gear_ratio(void) {
        return this->sensor_data.gear_ratio * 100.0F;
    }
    uint16_t get_targ_gear_ratio(void) {
        return this->sensor_data.targ_gear_ratio * 100.0F;
    }
    uint16_t redline_rpm;
    bool shifting = false;
    PressureManager* pressure_mgr = nullptr;

    bool isShifting(void) { return this->shifting; }
    uint8_t get_targ_curr_gear(void) { return (((uint8_t)this->target_gear) & 0x0F) << 4 | ((uint8_t)this->actual_gear & 0x0F); }
    uint8_t get_profile_id(void) {
        if (this->current_profile) {
            return this->current_profile->get_profile_id();
        } else {
            return 0xFF;
        }
    }
    TorqueConverter* tcc = nullptr;
    ShiftAlgoFeedback algo_feedback = {0};
    ShiftAdaptationSystem* shift_adapter = nullptr;
    SpeedSensors speed_sensors;    
private:
    bool is_stationary();
    ShiftReportSegment collect_report_segment(uint64_t start_time);
    void set_torque_request(TorqueRequestControlType ctrl_type, TorqueRequestBounds bounds, float amount);
    bool elapse_shift(GearChange req_lookup, AbstractProfile* profile, bool manually_requested);
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
    bool manual_shift = false;
    bool shift_req_was_manual = false;
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
    ShiftCircuit last_shift_circuit = ShiftCircuit::None;
    float diff_ratio_f =  1.0;
    GearChange shift_idx = GearChange::_IDLE;
    bool abort_shift = false;
    bool aborting = false;
    GearboxGear restrict_target = GearboxGear::Fifth;
    GearboxGear last_motion_gear = GearboxGear::Second;
    FirstOrderAverage* pedal_average = nullptr;
    FirstOrderAverage* motor_speed_average = nullptr;
    FirstOrderAverage* torque_req_average = nullptr;

    int req_static_torque_delta = 0;
    bool freeze_torque = false;

    KickdownSwitch kickdown;
    BrakePedal brake_pedal;

    bool is_start_safe = false;
};

extern Gearbox* gearbox;

#endif