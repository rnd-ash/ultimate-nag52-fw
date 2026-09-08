#include "shift_trace.h"
#include "models/vehicle_geometry.h"
#include "tcu_alloc.h"
#include "clock.hpp"
#include "esp_log.h"
#include "nvs/eeprom_config.h"
#include "egs_calibration/calibration_structs.h"
#include <math.h>
#include <string.h>

static ShiftTraceHeader trace_header = {};
static ShiftTraceSample* trace_ring = nullptr;
static bool was_shifting = false;


/**
 * @brief Running state for the shift quality metrics.
 *
 * Accumulated one sample at a time so it costs O(1) per control loop iteration
 * and needs no allocation. See ShiftQuality in the header for what each number
 * means and why they are a vector rather than a score.
 */
/**
 * @brief Samples of output speed kept for the derivative baseline.
 *
 * Output speed is an integer rpm, so differentiating it twice over one 20 ms
 * step has a floor: one rpm of wobble is 1/dt^2 * mps_per_output_rpm, which on
 * this car is 29.7 m/s^3 - above the whole comfort range. The 2026-09-08 drive
 * measured a median peak_jerk of 30.5 against that floor of 29.7, i.e. the
 * metric was reporting its own quantisation and not the shift. Acceleration had
 * the same problem: its floor is 53 rpm/s and torque_hole was reading 52.
 *
 * Widening the baseline to three samples divides the floor by nine (3.3 m/s^3)
 * and still resolves an inertia phase, which lasts 100-200 ms against this
 * 57 ms window. Measured over the same 39 shifts the median becomes 6.6 m/s^3,
 * which is inside the published comfort range and therefore actually
 * discriminates between a smooth shift and a harsh one.
 */
#define QUALITY_DERIV_SPAN 3
#define QUALITY_HIST (2 * QUALITY_DERIV_SPAN + 1)

static struct {
    uint32_t t_start;
    float ratio_start;
    float ratio_target;
    float accel_base;       // output shaft accel before the shift, rpm/s
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

// Vehicle longitudinal speed per rpm of output shaft, m/s. Set once at init.
// The applying clutch only starts to transmit once its pressure beats the return
// spring, so slip before that dissipates nothing. The calibration puts the
// springs at 1139-1289 mBar on this box.
#define QUALITY_SPRING_MBAR 1300
// Input shaft inertia, kg m^2, for the clutch torque estimate. Fitted from 302
// logged inertia phase samples across four drives.
#define QUALITY_INPUT_INERTIA 0.16f

void ShiftTrace::init(void) {
    // PSRAM: 13 KB is nothing there, and keeping it out of internal RAM leaves
    // the shift task's headroom alone.
    // TCU_HEAP_ALLOC is already the PSRAM heap on ESP32.
    trace_ring = static_cast<ShiftTraceSample*>(
        TCU_HEAP_ALLOC(SHIFT_TRACE_CAPACITY * sizeof(ShiftTraceSample)));
    if (nullptr == trace_ring) {
        // Not fatal - the TCU drives fine without a trace, so say so and move on.
        ESP_LOG_LEVEL(ESP_LOG_WARN, "TRACE", "Shift trace buffer allocation failed, tracing disabled");
        return;
    }
    memset(trace_ring, 0x00, SHIFT_TRACE_CAPACITY * sizeof(ShiftTraceSample));
    trace_header.magic = SHIFT_TRACE_MAGIC;
    trace_header.version = SHIFT_TRACE_VERSION;
    trace_header.sample_size = (uint8_t)sizeof(ShiftTraceSample);
    trace_header.capacity = (uint16_t)SHIFT_TRACE_CAPACITY;
    trace_header.buffer_addr = (uint32_t)trace_ring;
    trace_header.seq = 0;
    trace_header.dropped = 0;
    trace_header.n_events = 0;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "TRACE", "Shift trace ready: %u samples x %u bytes at 0x%08X (%u ms of history)",
        (unsigned)SHIFT_TRACE_CAPACITY, (unsigned)sizeof(ShiftTraceSample),
        (unsigned)trace_header.buffer_addr, (unsigned)(SHIFT_TRACE_CAPACITY * 20u));
}

const ShiftTraceHeader* ShiftTrace::get_header(void) {
    return (nullptr == trace_ring) ? nullptr : &trace_header;
}

/**
 * @brief Start a new event, retiring the oldest if the list is full.
 *
 * Events are kept newest-last so the host can walk them in order. Four is
 * enough: a shift's window is ~2.3 s and draining it costs ~72 ms, while the
 * shortest gap between shifts seen on the road is ~1.9 s.
 */
static void push_event(uint32_t seq, uint8_t from, uint8_t to, uint8_t pedal) {
    if (trace_header.n_events == SHIFT_TRACE_EVENTS) {
        // Oldest event is about to be lost. If the host never read it, its
        // samples are long gone from the ring too - count it so a gap in the
        // recorded shifts is visible rather than silent.
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
    e->pedal_start = pedal;
    memset(&e->quality, 0, sizeof(ShiftQuality));
    trace_header.n_events += 1;
}

void ShiftTrace::sample(const SensorData* sd, const ShiftAlgoFeedback* algo, bool shifting,
                        uint8_t gear_actual, uint8_t gear_target, uint16_t spc, uint16_t mpc,
                        uint8_t circuit_flags, int16_t trq_req_amount, int16_t engine_torque) {
    if (nullptr == trace_ring || nullptr == sd || nullptr == algo) {
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

    // Objective shift quality, accumulated as the shift runs.
    //
    // Both derivatives are taken over QUALITY_DERIV_SPAN samples rather than one,
    // because output speed is quantised to 1 rpm and a single-step second
    // difference measures that quantum and nothing else. See the note above.
    for (uint8_t i = 0; i < QUALITY_HIST - 1; i++) {
        q.out_hist[i] = q.out_hist[i + 1];
        q.t_hist[i] = q.t_hist[i + 1];
    }
    q.out_hist[QUALITY_HIST - 1] = s->output_rpm;
    q.t_hist[QUALITY_HIST - 1] = s->t_ms;
    if (q.hist_n < QUALITY_HIST) { q.hist_n += 1; }

    float accel = q.accel_prev;
    bool accel_ok = false;
    if (QUALITY_HIST == q.hist_n) {
        const uint8_t MID = QUALITY_DERIV_SPAN;
        const uint8_t END = QUALITY_HIST - 1;
        float dt_new = (float)(q.t_hist[END] - q.t_hist[MID]) / 1000.0f;
        float dt_old = (float)(q.t_hist[MID] - q.t_hist[0]) / 1000.0f;
        // A gap means the ring straddles a stall or a dropped cycle; skip it
        // rather than report the gap as a huge acceleration.
        if (dt_new > 0.0f && dt_old > 0.0f && dt_new < 0.25f && dt_old < 0.25f) {
            accel = ((float)q.out_hist[END] - (float)q.out_hist[MID]) / dt_new;
            float accel_older = ((float)q.out_hist[MID] - (float)q.out_hist[0]) / dt_old;
            // The two accelerations are centred half a window apart.
            float dt_jerk = (dt_new + dt_old) / 2.0f;
            accel_ok = true;
            if (shifting || q.settling) {
                float jerk = fabsf(accel - accel_older) / dt_jerk * mps_per_output_rpm();
                if (jerk > q.peak_jerk) { q.peak_jerk = jerk; }
            }
        }
    }
    uint32_t dt_ms = (q.t_prev == 0) ? 0 : (s->t_ms - q.t_prev);
    if (dt_ms > 0 && dt_ms < 200) {
        float dt = dt_ms / 1000.0f;
        if (shifting) {
            if (accel_ok && accel < q.min_accel) { q.min_accel = accel; }
            int32_t slip = (s->p_on > QUALITY_SPRING_MBAR) ? abs(algo->s_on) : -1;
            if (slip >= 0 && q.slip_prev >= 0) {
                float dw = ((float)s->input_rpm - (float)q.in_prev) / dt;
                float t_clutch = fabsf(QUALITY_INPUT_INERTIA * dw * 0.10472f) +
                                 fabsf((float)sd->input_torque);
                q.energy += t_clutch * ((float)slip * 0.10472f) * dt;
                if (q.slip_prev > slip) {
                    uint16_t rate = (uint16_t)((q.slip_prev - slip) / dt);
                    if (rate > q.lockup_rate) { q.lockup_rate = rate; }
                }
            }
            q.slip_prev = slip;
            // Response: how long before the ratio actually starts to move.
            if (0 == q.response_ms && s->output_rpm > 150 && q.ratio_target > 0.0f) {
                float r = (float)s->input_rpm / (float)s->output_rpm;
                float span = q.ratio_target - q.ratio_start;
                if (span != 0.0f && fabsf((r - q.ratio_start) / span) > 0.10f) {
                    q.response_ms = (uint16_t)(s->t_ms - q.t_start);
                }
            }
        } else if (q.settling) {
            // Driveline ringing after engagement: reversals about the pre-shift level
            if (accel_ok && ((q.accel_prev - q.accel_base) * (accel - q.accel_base)) < 0.0f && q.osc < 255) {
                q.osc += 1;
            }
            if (s->t_ms - q.t_start > 600u + (uint32_t)trace_header.events[trace_header.n_events - 1].quality.duration_ms) {
                q.settling = false;
            }
        }
        if (accel_ok) { q.accel_prev = accel; }
    }

    // Shift boundaries are detected here rather than hooked into elapse_shift,
    // so the shift control path is untouched.
    if (shifting && !was_shifting) {
        push_event(trace_header.seq, gear_actual, gear_target, s->pedal);
        // Reset the accumulators and latch the starting conditions.
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
            q.settling = true;      // keep watching for driveline ringing
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
