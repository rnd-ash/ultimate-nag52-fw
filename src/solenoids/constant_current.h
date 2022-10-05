#ifndef __CONSTANT_CURRENT_H_
#define __CONSTANT_CURRENT_H_

#include "solenoids.h"

class ConstantCurrentDriver {
public:
    ConstantCurrentDriver(Solenoid* target, const char *name);
    void set_target_current(uint16_t current);
    void toggle_state(bool enable);
    void update();
    float get_adjustment();
private:
    float pwm_adjustment_percent = 1.0;
    uint16_t current_target;
    Solenoid* solenoid;
    bool is_cc_mode;
    uint64_t last_off_time = 0;
    const char *name;
    uint64_t last_change_time = 0;
};


extern ConstantCurrentDriver* mpc_cc;
extern ConstantCurrentDriver* spc_cc;

namespace CurrentDriver {
    bool init_current_driver();
}

#endif