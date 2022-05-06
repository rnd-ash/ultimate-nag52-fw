// Here we go, gearbox controller code! Lets go!

#ifndef __GEARBOX_H_
#define __GEARBOX_H_

#include <stdint.h>
#include "canbus/can_hal.h"
#include "solenoids/solenoids.h"
#include "sensors.h"
#include "profiles.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common_structs.h"
#include <gearbox_config.h>
#include "torque_converter.h"
#include "behaviour/driving_profiler.h"
#include "pressure_manager.h"

// TODO Auto-set these based on CAN data about engine type
// 4000 is safe for now as it stops us over-revving diesel!
#define STALL_RPM 700
#define MIN_WORKING_RPM 1000

#define OVERSPEED_RPM 15000

#define ATF_TEMP_SAMPLES 20
struct TempSampleData {
    int samples[ATF_TEMP_SAMPLES];
    uint64_t total;
    uint8_t sample_id;
};

#define MAX_LIMIT 0.10 // 10% drift

const static GearRatioLimit GEAR_RATIO_LIMITS_SMALL[7] {
    GearRatioLimit { .max = RAT_1_SMALL*(1.0+MAX_LIMIT), .min = RAT_1_SMALL*(1.0-MAX_LIMIT) }, // 1
    GearRatioLimit { .max = RAT_2_SMALL*(1.0+MAX_LIMIT), .min = RAT_2_SMALL*(1.0-MAX_LIMIT) }, // 2
    GearRatioLimit { .max = RAT_3_SMALL*(1.0+MAX_LIMIT), .min = RAT_3_SMALL*(1.0-MAX_LIMIT) }, // 3
    GearRatioLimit { .max = RAT_4_SMALL*(1.0+MAX_LIMIT), .min = RAT_4_SMALL*(1.0-MAX_LIMIT) }, // 4
    GearRatioLimit { .max = RAT_5_SMALL*(1.0+MAX_LIMIT/2), .min = RAT_5_SMALL*(1.0-MAX_LIMIT/2) }, // 5 (Half tolorance so no overlap with 4th gear)
    GearRatioLimit { .max = RAT_R1_SMALL*(1.0-MAX_LIMIT), .min = RAT_R1_SMALL*(1.0+MAX_LIMIT) }, // R1
    GearRatioLimit { .max = RAT_R2_SMALL*(1.0-MAX_LIMIT), .min = RAT_R2_SMALL*(1.0+MAX_LIMIT) }, // R2
};

const static GearRatioLimit GEAR_RATIO_LIMITS_LARGE[7] {
    GearRatioLimit { .max = RAT_1_LARGE*(1.0+MAX_LIMIT), .min = RAT_1_LARGE*(1.0-MAX_LIMIT) }, // 1
    GearRatioLimit { .max = RAT_2_LARGE*(1.0+MAX_LIMIT), .min = RAT_2_LARGE*(1.0-MAX_LIMIT) }, // 2
    GearRatioLimit { .max = RAT_3_LARGE*(1.0+MAX_LIMIT), .min = RAT_3_LARGE*(1.0-MAX_LIMIT) }, // 3
    GearRatioLimit { .max = RAT_4_LARGE*(1.0+MAX_LIMIT), .min = RAT_4_LARGE*(1.0-MAX_LIMIT) }, // 4
    GearRatioLimit { .max = RAT_5_LARGE*(1.0+MAX_LIMIT/2), .min = RAT_5_LARGE*(1.0-MAX_LIMIT/2) }, // 5 (Half tolorance so no overlap with 4th gear)
    GearRatioLimit { .max = RAT_R1_LARGE*(1.0-MAX_LIMIT), .min = RAT_R1_LARGE*(1.0+MAX_LIMIT) }, // R1
    GearRatioLimit { .max = RAT_R2_LARGE*(1.0-MAX_LIMIT), .min = RAT_R2_LARGE*(1.0+MAX_LIMIT) }, // R2
};

const static FwdRatios RATIOS_LARGE {
    RAT_1_LARGE,
    RAT_2_LARGE,
    RAT_3_LARGE,
    RAT_4_LARGE,
    RAT_5_LARGE,
    RAT_R1_LARGE,
    RAT_R2_LARGE
};

const static FwdRatios RATIOS_SMALL {
    RAT_1_SMALL,
    RAT_2_SMALL,
    RAT_3_SMALL,
    RAT_4_SMALL,
    RAT_5_SMALL,
    RAT_R1_SMALL,
    RAT_R2_SMALL
};

class Gearbox {
public:
    Gearbox();
    void set_profile(AbstractProfile* prof);
    void inc_subprofile();
    bool start_controller();
    void inc_gear_request();
    void dec_gear_request();
    void diag_inhibit_control() { this->diag_stop_control = true; }
    void diag_regain_control() { this->diag_stop_control = false; }
    SensorData sensor_data;
        uint16_t get_gear_ratio() {
        return this->sensor_data.gear_ratio * 100;
    }
    static uint16_t redline_rpm;
private:
    ShiftResponse elapse_shift(ProfileGearChange req_lookup, AbstractProfile* profile, bool is_upshift);
    bool calcGearFromRatio(bool is_reverse);

    AbstractProfile* current_profile = nullptr;
    portMUX_TYPE profile_mutex;
    GearboxGear target_gear = GearboxGear::Park;
    GearboxGear actual_gear = GearboxGear::Park;
    GearboxGear last_fwd_gear = GearboxGear::Second;
    bool calc_input_rpm(int* dest);
    bool calc_output_rpm(int* dest, uint64_t now);
    [[noreturn]]
    void controller_loop();

    void shift_thread();
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
    bool shifting = false;
    bool ask_upshift = false;
    bool ask_downshift = false;
    float tcc_percent = 0;
    uint8_t est_gear_idx = 0;
    uint16_t curr_hold_pressure = 0;
    bool show_upshift = false;
    bool show_downshift = false;
    bool flaring = false;
    int gear_disagree_count = 0;
    unsigned long last_tcc_adjust_time = 0;
    int mpc_offset = 0;
    int mpc_working = 0;
    TorqueConverter* tcc = nullptr;
    TempSampleData temp_data;
    bool diag_stop_control = false;
    PressureManager* pressure_mgr = nullptr;
    ShifterPosition shifter_pos = ShifterPosition::SignalNotAvaliable;
    GearboxConfiguration gearboxConfig;
    float diff_ratio_f;
};

#endif