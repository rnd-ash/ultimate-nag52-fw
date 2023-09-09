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
#include "adaptation/shift_report.h"
#include "models/input_torque.hpp"
#include "adaptation/shift_adaptation.h"
#include "models/clutch_speed.hpp"

struct PostShiftTorqueRamp {
    bool enabled;
    uint16_t start_nm;
    uint16_t time_to_exit;
};

class Gearbox {
public:
    Gearbox(void);
    // Diag test
    ClutchSpeeds diag_get_clutch_speeds();
    void set_profile(AbstractProfile* prof);
    void inc_subprofile(void);
    esp_err_t start_controller(void);
    void inc_gear_request(void);
    void dec_gear_request(void);
    void diag_inhibit_control(void) { this->diag_stop_control = true; }
    void diag_regain_control(void) { this->diag_stop_control = false; }
    SensorData sensor_data;
    OutputData output_data;
    uint16_t get_gear_ratio(void) {
        return this->sensor_data.gear_ratio * 100.0F;
    }
    uint16_t redline_rpm;
    ShiftReporter* shift_reporter;
    bool shifting = false;
    PressureManager* pressure_mgr = nullptr;

    bool isShifting(void) { return this->shifting; }
    ProfileGearChange get_curr_gear_change(void) { return this->shift_idx; }
    TorqueConverter* tcc = nullptr;
    ShiftClutchVelocity shifting_velocity = {0,0};
    ShiftAdaptationSystem* shift_adapter = nullptr;
private:
    bool is_stationary();
    ShiftReportSegment collect_report_segment(uint64_t start_time);
    void set_torque_request(TorqueRequestControlType ctrl_type, TorqueRequestBounds bounds, float amount);
    bool elapse_shift(ProfileGearChange req_lookup, AbstractProfile* profile);
    bool calcGearFromRatio(bool is_reverse);

    AbstractProfile* current_profile = nullptr;
    portMUX_TYPE profile_mutex;
    GearboxGear target_gear = GearboxGear::Park;
    GearboxGear actual_gear = GearboxGear::Park;
    GearboxGear last_fwd_gear = GearboxGear::Second;
    bool calc_input_rpm(uint16_t* dest);
    bool calc_output_rpm(uint16_t* dest);
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
    ShifterPosition shifter_pos = ShifterPosition::SignalNotAvailable;
    GearboxConfiguration gearboxConfig;
    ShiftCircuit last_shift_circuit;
    float diff_ratio_f;
    ProfileGearChange shift_idx = ProfileGearChange::ONE_TWO;
    bool abort_shift = false;
    bool aborting = false;
    // Shadow ratios. These are calculated via the raw values from the speed sensors.
    // This way the TCU can see if a sensor is malfunctioning
    float shadow_ratio_n2 = 0;
    float shadow_ratio_n3 = 0;
    RpmReading rpm_reading;
    InputTorqueModel* itm;
    GearboxGear restrict_target = GearboxGear::Fifth;
    GearboxGear last_motion_gear = GearboxGear::Second;
    int calc_torque_limit(ProfileGearChange change, uint16_t shift_speed_ms);
    MovingAverage* output_avg_filter;

};

extern Gearbox* gearbox;

#endif