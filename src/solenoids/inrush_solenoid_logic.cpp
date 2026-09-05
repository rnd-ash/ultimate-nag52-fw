#include "inrush_solenoid_logic.h"

#include <algorithm>

InrushHoldStepPlan inrush_plan_hold_step(
    uint32_t pwm_on_time_us,
    uint32_t pwm_off_time_us,
    bool phase_is_pwm_off,
    uint32_t total_elapsed_us,
    uint32_t hold_time_us
) {
    const uint32_t phase_duration = phase_is_pwm_off ? pwm_off_time_us : pwm_on_time_us;
    const uint32_t remaining = (total_elapsed_us >= hold_time_us)
        ? 0u
        : (hold_time_us - total_elapsed_us);
    uint32_t step = std::min(phase_duration, remaining);
    if (step == 0u && remaining != 0u) {
        // Never schedule a zero-length timer step while hold is still active.
        step = remaining;
    }
    const uint32_t next_total = total_elapsed_us + step;

    return InrushHoldStepPlan{
        .step_us = step,
        .next_total_us = next_total,
        .hold_completed = (next_total >= hold_time_us),
    };
}
