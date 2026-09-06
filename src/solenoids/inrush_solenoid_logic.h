#ifndef INRUSH_SOLENOID_LOGIC_H
#define INRUSH_SOLENOID_LOGIC_H

#include <stdint.h>

// inrush_plan_hold_step() is called from InrushControlSolenoid::on_timer_interrupt_new(),
// which runs in the TCC solenoid gptimer ISR. That ISR is IRAM safe
// (CONFIG_GPTIMER_ISR_IRAM_SAFE), so it is still serviced while a flash write
// has the CPU cache disabled - and a flash resident (IROM) function CANNOT be
// called in that window without panicking the TCU. This helper must therefore
// live in IRAM. See the ISR CACHE-SAFETY CONTRACT at the top of
// src/solenoids/inrush_solenoid.cpp.
//
// Host builds have no IRAM, so the attribute collapses to nothing there and the
// logic stays unit testable.
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define INRUSH_LOGIC_IRAM IRAM_ATTR
#else
#define INRUSH_LOGIC_IRAM
#endif

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
