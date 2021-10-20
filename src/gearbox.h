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

class Gearbox {
public:
    Gearbox();
    void set_profile(AbstractProfile* prof);
    bool start_controller();
private:
    AbstractProfile* current_profile = nullptr;
    portMUX_TYPE profile_mutex;
    GearboxGear target_gear = GearboxGear::SignalNotAvaliable;
    GearboxGear actual_gear = GearboxGear::SignalNotAvaliable;
    bool calc_input_rpm(uint32_t* dest);
    bool calc_output_rpm(int* dest);
    [[noreturn]]
    void controller_loop();

    void shift_thread();

    static void start_shift_thread(void *_this) {
        static_cast<Gearbox*>(_this)->shift_thread();
    }

    [[noreturn]]
    static void start_controller_internal(void *_this) {
        static_cast<Gearbox*>(_this)->controller_loop();
    }

    TaskHandle_t shift_task = nullptr;
    bool shifting = false;
};

#endif