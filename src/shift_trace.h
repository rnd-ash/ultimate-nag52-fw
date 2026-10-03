#ifndef SHIFT_TRACE_H
#define SHIFT_TRACE_H

#include <stdint.h>
#include "common_structs.h"

#define SHIFT_TRACE_MAGIC 0x43415254u
#define SHIFT_TRACE_VERSION 3u
#define SHIFT_TRACE_CAPACITY 512u
#define SHIFT_TRACE_EVENTS 4u

struct ShiftTraceSample {
    uint32_t t_ms;
    uint16_t input_rpm;
    uint16_t output_rpm;
    uint16_t engine_rpm;
    int16_t input_torque;
    uint16_t p_on;
    uint16_t p_off;
    uint16_t spc;
    uint16_t mpc;
    uint8_t phase;
    uint8_t subphase_shift;
    uint8_t subphase_mod;
    uint8_t flags;
    uint8_t pedal;
    uint8_t gear;
    int16_t trq_req_amount;
    int16_t engine_torque;
} __attribute__((packed));

struct ShiftQuality {
    uint16_t response_ms;
    uint16_t duration_ms;
    uint16_t peak_jerk;
    uint16_t torque_hole;
    uint32_t slip_energy_j;
    uint16_t lockup_rate;
    uint8_t settle_osc;
    uint8_t valid;
} __attribute__((packed));

struct ShiftStamp {
    uint8_t features;
    uint8_t arm;
    uint8_t blend_pct;
    uint8_t flags;
    uint8_t adapt_reason;
    uint8_t algorithm;
    uint16_t target_time_ms;
    int16_t spc_offset;
    int16_t prefill_offset;
    int16_t spc_delta;
    int16_t prefill_delta;
} __attribute__((packed));

struct ShiftTraceEvent {
    uint32_t seq_start;
    uint32_t seq_end;
    uint8_t gear_from;
    uint8_t gear_to;
    uint8_t done;
    uint8_t agility_score;
    ShiftQuality quality;
    ShiftStamp stamp;
} __attribute__((packed));

struct ShiftTraceHeader {
    uint32_t magic;
    uint8_t version;
    uint8_t sample_size;
    uint16_t capacity;
    uint32_t buffer_addr;
    uint32_t seq;
    uint32_t dropped;
    uint8_t n_events;
    uint8_t _pad[3];
    ShiftTraceEvent events[SHIFT_TRACE_EVENTS];
} __attribute__((packed));

static_assert(sizeof(ShiftTraceSample) == 30, "ShiftTraceSample must stay 30 bytes");
static_assert(sizeof(ShiftQuality) == 16, "ShiftQuality must stay 16 bytes");
static_assert(sizeof(ShiftStamp) == 16, "ShiftStamp must stay 16 bytes");
static_assert(sizeof(ShiftTraceEvent) == 44, "ShiftTraceEvent must stay 44 bytes");
static_assert(sizeof(ShiftTraceHeader) == 24 + (44 * SHIFT_TRACE_EVENTS), "ShiftTraceHeader layout changed");

namespace ShiftTrace {
    void init(void);
    bool set_enabled(bool enabled);
    bool is_enabled(void);
    void sample(const SensorData* sd, const ShiftAlgoFeedback* algo, bool shifting,
                uint8_t gear_actual, uint8_t gear_target, uint16_t spc, uint16_t mpc,
                uint8_t circuit_flags, int16_t trq_req_amount, int16_t engine_torque);
    const ShiftTraceHeader* get_header(void);
}

#endif
