#pragma once
#include <stdint.h>
#ifndef IRAM_ATTR
#include "esp_attr.h"
#endif

// All durations are 10 MHz timer ticks. This state machine owns one complete
// hydraulic cycle; a task cannot change the duration of a pulse already running.
struct TccPulse {
    static constexpr uint32_t period = 100000;
    struct Command {
        uint32_t inrush = 0, hold = 0, off = period;
        uint32_t on_ticks = 3500, off_ticks = 6500;
        uint16_t duty = 0;
    } command, cycle;
    struct Edge { uint32_t ticks; uint16_t duty; bool zener; };
    uint8_t phase = 0;
    uint32_t elapsed = 0;
    bool hold_on = false;

    void IRAM_ATTR reset() { phase = 0; elapsed = 0; hold_on = false; }

    Edge IRAM_ATTR next(bool gpio_mode) {
        if (command.duty == 0 || (!gpio_mode && command.duty < 224)) {
            reset();
            return {period, 0, false};
        }
        if (!gpio_mode && command.duty > 3220) {
            reset();
            return {period, 1024, false};
        }
        // Skip empty phases without ever arming a zero-length alarm.
        for (unsigned n = 0; n < 3; ++n) {
            if (phase == 0) {
                cycle = command;
                elapsed = 0;
                hold_on = false;
                phase = 1;
                if (cycle.inrush) return {cycle.inrush, 4096, false};
            } else if (phase == 1) {
                if (elapsed < cycle.hold) {
                    uint32_t ticks = cycle.hold - elapsed;
                    uint16_t duty = 1024;
                    if (gpio_mode) {
                        const uint32_t carrier = hold_on ? cycle.on_ticks : cycle.off_ticks;
                        if (ticks > carrier) ticks = carrier;
                        duty = hold_on ? 4096 : 0;
                        hold_on = !hold_on;
                    }
                    // Even a malformed carrier must not publish an energized
                    // edge with no future deadline to turn it off.
                    if (ticks == 0) {
                        reset();
                        return {period, 0, false};
                    }
                    elapsed += ticks;
                    if (elapsed == cycle.hold) phase = 2;
                    return {ticks, duty, false};
                }
                phase = 2;
            } else {
                phase = 0;
                if (cycle.off) return {cycle.off, 0, gpio_mode};
            }
        }
        // Defensive off interval for malformed commands. Never latch an on edge.
        reset();
        return {period, 0, false};
    }
};
