#include "shift_trace.h"

#include <math.h>
#include <string.h>

#include "clock.hpp"
#include "egs_calibration/calibration_structs.h"
#include "esp_log.h"
#include "models/vehicle_geometry.h"
#include "tcu_maths.h"
#include "tcu_alloc.h"

static ShiftTraceHeader trace_header = {};
static ShiftTraceSample* trace_ring = nullptr;
static bool trace_enabled = false;
static bool was_shifting = false;

#define QUALITY_DERIV_SPAN 3
#define QUALITY_HIST (2 * QUALITY_DERIV_SPAN + 1)
#define QUALITY_SPRING_MBAR 1300
#define QUALITY_INPUT_INERTIA 0.16f

static struct {
    uint32_t t_start;
    float ratio_start;
    float ratio_target;
    float accel_base;
    float accel_prev;
    uint16_t out_hist[QUALITY_HIST];
    uint32_t t_hist[QUALITY_HIST];
    uint8_t hist_n;
    uint16_t out_prev;
    uint16_t in_prev;
    uint32_t t_prev;
    int32_t slip_prev;
    float energy;
    float peak_jerk;
    float min_accel;
    uint16_t response_ms;
    uint16_t lockup_rate;
    uint8_t osc;
    bool settling;
} q = {};

static bool ensure_allocated(void) {
    if (nullptr != trace_ring) {
        return true;
    }
    trace_ring = static_cast<ShiftTraceSample*>(
        TCU_HEAP_ALLOC(SHIFT_TRACE_CAPACITY * sizeof(ShiftTraceSample)));
    if (nullptr == trace_ring) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "TRACE", "Shift trace allocation failed; recorder remains disabled");
        return false;
    }
    memset(trace_ring, 0x00, SHIFT_TRACE_CAPACITY * sizeof(ShiftTraceSample));
    trace_header.magic = SHIFT_TRACE_MAGIC;
    trace_header.version = SHIFT_TRACE_VERSION;
    trace_header.sample_size = (uint8_t)sizeof(ShiftTraceSample);
    trace_header.capacity = (uint16_t)SHIFT_TRACE_CAPACITY;
    trace_header.buffer_addr = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(trace_ring));
    trace_header.seq = 0;
    trace_header.dropped = 0;
    trace_header.n_events = 0;
    return true;
}

void ShiftTrace::init(void) {
    trace_enabled = false;
}

bool ShiftTrace::set_enabled(bool enabled) {
    if (enabled) {
        if (!ensure_allocated()) {
            trace_enabled = false;
            return false;
        }
        trace_enabled = true;
    } else {
        trace_enabled = false;
        // A disabled interval has no samples: restart event/derivative history on re-enable.
        was_shifting = false;
        q = {};
    }
    return true;
}

bool ShiftTrace::is_enabled(void) {
    return trace_enabled;
}

const ShiftTraceHeader* ShiftTrace::get_header(void) {
    if (!trace_enabled || nullptr == trace_ring) {
        return nullptr;
    }
    return &trace_header;
}

static void push_event(uint32_t seq, uint8_t from, uint8_t to, uint8_t agility) {
    if (trace_header.n_events == SHIFT_TRACE_EVENTS) {
        if (0 == trace_header.events[0].done ||
            (trace_header.seq - trace_header.events[0].seq_start) >= SHIFT_TRACE_CAPACITY) {
            trace_header.dropped += 1;
        }
        memmove(&trace_header.events[0], &trace_header.events[1],
                sizeof(ShiftTraceEvent) * (SHIFT_TRACE_EVENTS - 1));
        trace_header.n_events -= 1;
    }
    ShiftTraceEvent* e = &trace_header.events[trace_header.n_events];
    e->seq_start = seq;
    e->seq_end = seq;
    e->gear_from = from;
    e->gear_to = to;
    e->done = 0;
    e->agility_score = agility;
    memset(&e->quality, 0, sizeof(ShiftQuality));
    memset(&e->stamp, 0, sizeof(ShiftStamp));
    e->stamp.algorithm = 1;
    trace_header.n_events += 1;
}

void ShiftTrace::sample(const SensorData* sd, const ShiftAlgoFeedback* algo, bool shifting,
                        uint8_t gear_actual, uint8_t gear_target, uint16_t spc, uint16_t mpc,
                        uint8_t circuit_flags, int16_t trq_req_amount, int16_t engine_torque) {
    if (!trace_enabled || nullptr == trace_ring || nullptr == sd || nullptr == algo) {
        return;
    }
    ShiftTraceSample* s = &trace_ring[trace_header.seq % SHIFT_TRACE_CAPACITY];
    s->t_ms = GET_CLOCK_TIME();
    s->input_rpm = sd->input_rpm;
    s->output_rpm = sd->output_rpm;
    s->engine_rpm = sd->engine_rpm;
    s->input_torque = sd->input_torque;
    s->p_on = algo->p_on;
    s->p_off = algo->p_off;
    s->spc = spc;
    s->mpc = mpc;
    s->phase = algo->shift_phase;
    s->subphase_shift = algo->subphase_shift;
    s->subphase_mod = algo->subphase_mod;
    s->flags = (shifting ? 0x01u : 0x00u) | (uint8_t)((circuit_flags & 0x0Fu) << 1);
    s->pedal = (sd->pedal_pos > 250) ? 250u : (uint8_t)sd->pedal_pos;
    s->gear = (uint8_t)((gear_actual & 0x0Fu) << 4 | (gear_target & 0x0Fu));
    s->trq_req_amount = trq_req_amount;
    s->engine_torque = engine_torque;

    for (uint8_t i = 0; i < QUALITY_HIST - 1; i++) {
        q.out_hist[i] = q.out_hist[i + 1];
        q.t_hist[i] = q.t_hist[i + 1];
    }
    q.out_hist[QUALITY_HIST - 1] = s->output_rpm;
    q.t_hist[QUALITY_HIST - 1] = s->t_ms;
    if (q.hist_n < QUALITY_HIST) {
        q.hist_n += 1;
    }

    float accel = q.accel_prev;
    bool accel_ok = false;
    if (QUALITY_HIST == q.hist_n) {
        const uint8_t mid = QUALITY_DERIV_SPAN;
        const uint8_t end = QUALITY_HIST - 1;
        float dt_new = (float)(q.t_hist[end] - q.t_hist[mid]) / 1000.0f;
        float dt_old = (float)(q.t_hist[mid] - q.t_hist[0]) / 1000.0f;
        if (dt_new > 0.0f && dt_old > 0.0f && dt_new < 0.25f && dt_old < 0.25f) {
            accel = ((float)q.out_hist[end] - (float)q.out_hist[mid]) / dt_new;
            float accel_older = ((float)q.out_hist[mid] - (float)q.out_hist[0]) / dt_old;
            float dt_jerk = (dt_new + dt_old) / 2.0f;
            accel_ok = true;
            if (shifting || q.settling) {
                float jerk = fabsf(accel - accel_older) / dt_jerk * mps_per_output_rpm();
                if (jerk > q.peak_jerk) {
                    q.peak_jerk = jerk;
                }
            }
        }
    }

    uint32_t dt_ms = (q.t_prev == 0) ? 0 : (s->t_ms - q.t_prev);
    if (dt_ms > 0 && dt_ms < 200) {
        float dt = dt_ms / 1000.0f;
        if (shifting) {
            if (accel_ok && accel < q.min_accel) {
                q.min_accel = accel;
            }
            int32_t slip = (s->p_on > QUALITY_SPRING_MBAR) ? abs(algo->s_on) : -1;
            if (slip >= 0 && q.slip_prev >= 0) {
                float dw = ((float)s->input_rpm - (float)q.in_prev) / dt;
                float t_clutch = fabsf(QUALITY_INPUT_INERTIA * dw * 0.10472f) +
                                 fabsf((float)sd->input_torque);
                q.energy += t_clutch * ((float)slip * 0.10472f) * dt;
                if (q.slip_prev > slip) {
                    uint16_t rate = (uint16_t)((q.slip_prev - slip) / dt);
                    if (rate > q.lockup_rate) {
                        q.lockup_rate = rate;
                    }
                }
            }
            q.slip_prev = slip;
            if (0 == q.response_ms && s->output_rpm > 150 && q.ratio_target > 0.0f) {
                float r = (float)s->input_rpm / (float)s->output_rpm;
                float span = q.ratio_target - q.ratio_start;
                if (span != 0.0f && fabsf((r - q.ratio_start) / span) > 0.10f) {
                    q.response_ms = (uint16_t)(s->t_ms - q.t_start);
                }
            }
        } else if (q.settling) {
            if (accel_ok && ((q.accel_prev - q.accel_base) * (accel - q.accel_base)) < 0.0f && q.osc < 255) {
                q.osc += 1;
            }
            if (s->t_ms - q.t_start > 600u + (uint32_t)trace_header.events[trace_header.n_events - 1].quality.duration_ms) {
                q.settling = false;
            }
        }
        if (accel_ok) {
            q.accel_prev = accel;
        }
    }

    if (shifting && !was_shifting) {
        push_event(trace_header.seq, gear_actual, gear_target, s->pedal);
        q.t_start = s->t_ms;
        q.ratio_start = (s->output_rpm > 150) ? ((float)s->input_rpm / (float)s->output_rpm) : 0.0f;
        q.ratio_target = 0.0f;
        if (gear_target >= 1 && gear_target <= 7 && MECH_PTR != nullptr) {
            q.ratio_target = (float)MECH_PTR->ratio_table[gear_target] / 1000.0f;
        }
        q.accel_base = q.accel_prev;
        q.energy = 0.0f;
        q.peak_jerk = 0.0f;
        q.min_accel = q.accel_prev;
        q.response_ms = 0;
        q.lockup_rate = 0;
        q.osc = 0;
        q.slip_prev = -1;
        q.settling = false;
    } else if (!shifting && was_shifting && trace_header.n_events > 0) {
        ShiftTraceEvent* e = &trace_header.events[trace_header.n_events - 1];
        if (0 == e->done) {
            e->seq_end = trace_header.seq;
            e->done = 1;
            e->quality.duration_ms = (uint16_t)MIN(65535u, s->t_ms - q.t_start);
            e->quality.response_ms = q.response_ms;
            e->quality.peak_jerk = (uint16_t)MIN(65535.0f, q.peak_jerk * 1000.0f);
            e->quality.torque_hole = (uint16_t)MIN(65535.0f, MAX(0.0f, q.accel_base - q.min_accel));
            e->quality.slip_energy_j = (uint32_t)MAX(0.0f, q.energy);
            e->quality.lockup_rate = q.lockup_rate;
            e->quality.settle_osc = 0;
            e->quality.valid = 1;
            q.settling = true;
        }
    }
    if (!shifting && !q.settling && trace_header.n_events > 0) {
        ShiftTraceEvent* e = &trace_header.events[trace_header.n_events - 1];
        if (e->quality.valid && e->quality.settle_osc == 0 && q.osc > 0) {
            e->quality.settle_osc = q.osc;
        }
    }

    q.out_prev = s->output_rpm;
    q.in_prev = s->input_rpm;
    q.t_prev = s->t_ms;
    was_shifting = shifting;
    trace_header.seq += 1;
}
