#ifndef SHIFT_TRACE_H
#define SHIFT_TRACE_H

#include <stdint.h>
#include "common_structs.h"

/**
 * @brief High rate shift recorder.
 *
 * The diagnostic link is request/response, so a host poll of the default nine
 * records takes 52.5 ms (measured median over 29944 cycles across four drives),
 * i.e. 19 Hz. The sensors and the shift algorithm both update every 20 ms, so
 * polling captured only one update in 2.6 - 62 % of them never left the TCU,
 * and which ones survived drifted arbitrarily against the shift. A shift's
 * inertia phase lasts 100-200 ms, leaving about three aliased samples: too few
 * to see how a shift went, let alone to fit a model to it.
 *
 * So the TCU records it itself. A ring in PSRAM is filled from
 * Gearbox::controller_loop at its native 20 ms period, which is also the period
 * ShiftingAlgorithm steps at (SHIFT_DELAY_MS), so the capture is lossless with
 * respect to the algorithm. The host reads the ring out afterwards, in the quiet
 * time between shifts.
 *
 * Nothing here runs inside the shift control path, and the sampler is O(1) with
 * no allocation.
 *
 * Readout: fetch ShiftTraceHeader from RLI_SHIFT_TRACE, then pull samples with
 * ReadMemoryByAddress using `buffer_addr` (each response carries at most 255
 * bytes and the host asks for one chunk at a time, so the USB serial bridge's
 * FIFO is never burst through). `seq` counts every sample ever written, so
 * sample n is still in the ring while `seq - n < capacity`, which is also how
 * the host detects an overrun.
 */

#define SHIFT_TRACE_MAGIC 0x43415254u  // 'TRAC'
#define SHIFT_TRACE_VERSION 1u
#define SHIFT_TRACE_CAPACITY 512u      // 512 * 20 ms = 10.2 s of history
#define SHIFT_TRACE_EVENTS 4u

struct ShiftTraceSample {
    uint32_t t_ms;          // GET_CLOCK_TIME() when sampled
    uint16_t input_rpm;
    uint16_t output_rpm;
    uint16_t engine_rpm;
    int16_t  input_torque;
    uint16_t p_on;          // on clutch pressure  (mBar)
    uint16_t p_off;         // off clutch pressure (mBar)
    uint16_t spc;           // corrected shift pressure      (mBar)
    uint16_t mpc;           // corrected modulating pressure (mBar)
    uint8_t  phase;         // ShiftingAlgorithm phase id
    uint8_t  subphase_shift;
    uint8_t  subphase_mod;
    uint8_t  flags;         // bit0 shifting, bits 1-4 shift circuit flags
    uint8_t  pedal;         // raw, 0-250
    uint8_t  gear;          // actual << 4 | target
    int16_t  trq_req_amount;// absolute torque asked of the engine, INT16_MAX = no request
    int16_t  engine_torque; // what the engine reports it is making (CAN static torque)
} __attribute__((packed));  // 30 bytes

/**
 * @brief Objective shift quality, computed on the TCU as the shift happens.
 *
 * There is no single number for shift quality and no mode-independent one:
 * Comfort wants low jerk and will pay for it in duration, Agility wants
 * spontaneity and accepts jerk to get it. So this is a vector, and what counts
 * as good has to be decided per driving mode by whoever consumes it.
 *
 * Nothing in the firmware acts on these - they are recorded so that a shift can be
 * judged from the car rather than only from an offline log, and so that any
 * future adaptation has a reward signal to learn against.
 *
 * Metrics follow the published ones: jerk is the measure that correlates with
 * subjective shift feel (SAE 650465), duration is reported with it because a
 * shift can always be made smooth by making it long, and slip energy is the
 * wear and thermal load the friction material has to absorb.
 */
struct ShiftQuality {
    uint16_t response_ms;   // request until the ratio actually starts to move
    uint16_t duration_ms;
    // mm/s^3 (m/s^3 x1000) of vehicle longitudinal jerk.
    //
    // SI, via mps_per_output_rpm(). Converting needs the wheel circumference and
    // final drive, which the TCU cannot verify - but the error is about 1 % for a
    // properly plus-sized wheel and 8 % for a 20 inch wheel nobody would fit,
    // against a metric that reads twice different between 19 Hz and 50 Hz
    // sampling. Worth it to keep the number comparable with the published
    // thresholds (comfortable under ~10, objectionable over ~20-30, SAE 650465)
    // and with whatever target a user sets.
    // Derivatives are taken over a 3 sample (57 ms) baseline, not one step: the
    // output speed is quantised to 1 rpm, and a single-step second difference
    // has a floor of 29.7 m/s^3 on this car, which is above the entire comfort
    // range. Before that was fixed this field reported its own quantisation.
    uint16_t peak_jerk;
    uint16_t torque_hole;   // rpm/s of output shaft accel lost mid-shift
    uint32_t slip_energy_j; // joules dissipated in the applying clutch
    uint16_t lockup_rate;   // rpm/s at which the applying clutch slip collapses
    uint8_t  settle_osc;    // driveline accel reversals after engagement
    uint8_t  valid;
} __attribute__((packed));  // 16 bytes

struct ShiftTraceEvent {
    uint32_t seq_start;     // sample index at which the shift began
    uint32_t seq_end;       // sample index at which it ended (valid when done)
    uint8_t  gear_from;
    uint8_t  gear_to;
    uint8_t  done;
    uint8_t  pedal_start;   // accelerator position 0-250 when the shift began
    ShiftQuality quality;
} __attribute__((packed));  // 28 bytes

struct ShiftTraceHeader {
    uint32_t magic;
    uint8_t  version;
    uint8_t  sample_size;
    uint16_t capacity;
    uint32_t buffer_addr;   // real address of sample 0, for ReadMemoryByAddress
    uint32_t seq;           // total samples written since boot
    uint32_t dropped;       // shifts whose window was overwritten before readout
    uint8_t  n_events;      // number of valid entries in `events`
    uint8_t  _pad[3];
    ShiftTraceEvent events[SHIFT_TRACE_EVENTS];
} __attribute__((packed));

// The host decoder (logger/nag52logger/shift_trace.py) unpacks these by size, and
// the header carries sample_size so a mismatch is reported rather than silently
// mis-decoded. Pin them here so the two cannot drift apart unnoticed.
static_assert(sizeof(ShiftTraceSample) == 30, "ShiftTraceSample must stay 30 bytes");
static_assert(sizeof(ShiftQuality) == 16, "ShiftQuality must stay 16 bytes");
static_assert(sizeof(ShiftTraceEvent) == 28, "ShiftTraceEvent must stay 28 bytes");
static_assert(sizeof(ShiftTraceHeader) == 24 + (28 * SHIFT_TRACE_EVENTS), "ShiftTraceHeader layout changed");

namespace ShiftTrace {
    /// Allocate the ring. Safe to fail - tracing is then simply inactive.
    void init(void);
    /// One sample. Called from Gearbox::controller_loop every 20 ms.
    void sample(const SensorData* sd, const ShiftAlgoFeedback* algo, bool shifting,
                uint8_t gear_actual, uint8_t gear_target, uint16_t spc, uint16_t mpc,
                uint8_t circuit_flags, int16_t trq_req_amount, int16_t engine_torque);
    /// Header for the diagnostic readout, or nullptr if tracing is inactive.
    const ShiftTraceHeader* get_header(void);

}

#endif
