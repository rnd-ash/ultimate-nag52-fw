#ifndef INRUSH_SOLENOID_LOGIC_H
#define INRUSH_SOLENOID_LOGIC_H

#include <stdint.h>

struct InrushHoldStepPlan {
    uint32_t step_us;
    uint32_t next_total_us;
    bool hold_completed;
};

InrushHoldStepPlan inrush_plan_hold_step(
    uint32_t pwm_on_time_us,
    uint32_t pwm_off_time_us,
    bool phase_is_pwm_off,
    uint32_t total_elapsed_us,
    uint32_t hold_time_us
);

#endif // INRUSH_SOLENOID_LOGIC_H
