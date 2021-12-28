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

// TODO Auto-set these based on CAN data about engine type
// 4000 is safe for now as it stops us over-revving diesel!
#define REDLINE_RPM 4000
#define STALL_RPM 700
#define MIN_WORKING_RPM 1000

#define OVERSPEED_RPM 15000

typedef struct {
    float max;
    float min;
} GearRatioLimit;

#define ATF_TEMP_SAMPLES 20
struct TempSampleData {
    int samples[ATF_TEMP_SAMPLES];
    uint64_t total;
    uint8_t sample_id;
};

#define MAX_LIMIT 0.10 // 10% drift

const static GearRatioLimit GEAR_RATIO_LIMITS[7] {
    GearRatioLimit { .max = RAT_1*(1.0+MAX_LIMIT), .min = RAT_1*(1.0-MAX_LIMIT) }, // 1
    GearRatioLimit { .max = RAT_2*(1.0+MAX_LIMIT), .min = RAT_2*(1.0-MAX_LIMIT) }, // 2
    GearRatioLimit { .max = RAT_3*(1.0+MAX_LIMIT), .min = RAT_3*(1.0-MAX_LIMIT) }, // 3
    GearRatioLimit { .max = RAT_4*(1.0+MAX_LIMIT), .min = RAT_4*(1.0-MAX_LIMIT) }, // 4
    GearRatioLimit { .max = RAT_5*(1.0+MAX_LIMIT), .min = RAT_5*(1.0-MAX_LIMIT) }, // 5
    GearRatioLimit { .max = RAT_R1*(1.0-MAX_LIMIT), .min = RAT_R1*(1.0+MAX_LIMIT) }, // R1
    GearRatioLimit { .max = RAT_R2*(1.0-MAX_LIMIT), .min = RAT_R2*(1.0+MAX_LIMIT) }, // R2
};

class Gearbox {
public:
    Gearbox();
    void set_profile(AbstractProfile* prof);
    void inc_subprofile();
    bool start_controller();
    void inc_gear_request();
    void dec_gear_request();
private:
    ShiftResponse elapse_shift(ProfileGearChange req_lookup, AbstractProfile* profile, Solenoid* shift_solenoid, uint8_t curr_gear, uint8_t targ_gear);
    bool calcGearFromRatio(bool is_reverse);

    AbstractProfile* current_profile = nullptr;
    portMUX_TYPE profile_mutex;
    GearboxGear target_gear = GearboxGear::Park;
    GearboxGear actual_gear = GearboxGear::Park;
    GearboxGear min_fwd_gear = GearboxGear::First;
    bool calc_input_rpm(int* dest);
    bool calc_output_rpm(int* dest, uint64_t now);
    [[noreturn]]
    void controller_loop();
    [[noreturn]]
    void torque_converter_loop();

    void shift_thread();
    bool start_second = true; // By default
    static void start_shift_thread(void *_this) {
        static_cast<Gearbox*>(_this)->shift_thread();
    }

    [[noreturn]]
    static void start_controller_internal(void *_this) {
        static_cast<Gearbox*>(_this)->controller_loop();
    }
    [[noreturn]]
    static void start_torque_converter(void *_this) {
        static_cast<Gearbox*>(_this)->torque_converter_loop();
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
    TorqueConverter* tcc = nullptr;
    SensorData sensor_data;
    TempSampleData temp_data;
};

#endif