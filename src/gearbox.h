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

class Gearbox {
public:
    Gearbox();
    void set_profile(AbstractProfile* prof);
    bool start_controller();
private:
    AbstractProfile* current_profile = nullptr;
    portMUX_TYPE profile_mutex;
    [[noreturn]]
    void controller_loop();
    [[noreturn]]
    static void start_controller_internal(void *_this) {
        static_cast<Gearbox*>(_this)->controller_loop();
    }
};

#endif