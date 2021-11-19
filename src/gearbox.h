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

// TODO Auto-set these based on CAN data about engine type
// 4000 is safe for now as it stops us over-revving diesel!
#define REDLINE_RPM 4000
#define STALL_RPM 700
#define MIN_WORKING_RPM 1000

#define OVERSPEED_RPM 10000

//#define LARGE_NAG

// https://en.wikipedia.org/wiki/Mercedes-Benz_5G-Tronic_transmission
#ifdef LARGE_NAG
    #define RAT_1 3.5876
    #define RAT_2 2.1862
    #define RAT_3 1.4054
    #define RAT_4 1.0000
    #define RAT_5 0.8314
    #define RAT_R1 -3.1605
    #define RAT_R2 -1.9259
#else
    #define RAT_1 3.9319
    #define RAT_2 2.4079
    #define RAT_3 1.4857
    #define RAT_4 1.0000
    #define RAT_5 0.8305
    #define RAT_R1 -3.1002
    #define RAT_R2 -1.8986
#endif

typedef struct {
    float max;
    float min;
} GearRatioLimit;

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
    uint16_t elapse_shift(uint16_t init_spc, uint16_t init_mpc, Solenoid* shift_solenoid, uint16_t target_shift_duration_ms, uint8_t targ_gear);
    bool calcGearFromRatio(uint32_t input_rpm, uint32_t output_rpm, bool is_reverse);

    AbstractProfile* current_profile = nullptr;
    portMUX_TYPE profile_mutex;
    GearboxGear target_gear = GearboxGear::Park;
    GearboxGear actual_gear = GearboxGear::Park;
    GearboxGear min_fwd_gear = GearboxGear::First;
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
    uint16_t tcc_percent = 0;
    uint8_t pedal_pos = 0;
    uint8_t est_gear_idx = 0;
};

#endif