#ifndef TCU_SCALING_H
#define TCU_SCALING_H

#include <stdint.h>

/**
 * @file tcu_scaling.h
 * @brief Named scaling for values that travel through the TCU in a raw,
 *        non-obvious unit.
 *
 * The rule this file exists to enforce: if a quantity is stored in anything
 * other than its natural unit, the conversion belongs here with a name, not
 * inline as a bare literal. A stray "/ 2.5f" tells the next reader nothing;
 * Pedal::to_percent() tells them everything.
 *
 * Header only and constexpr, so there is no runtime cost and it works
 * unchanged in host tests.
 */

/**
 * @brief Accelerator pedal demand.
 *
 * Pedal demand is carried through the whole TCU as a RAW value with a full
 * scale of 250 - the Mercedes CAN convention (MS_210h PW is 0.4% per bit).
 * It is NOT a percentage and NOT 0-255.
 *
 * Getting this wrong is not theoretical: HfmCan::get_pedal_value() used to
 * return 0-100, so on HFM cars every consumer read pedal demand at 40% of its
 * true value - wrong shift map cells, wrong TCC load, and the coast flag
 * tripping at the wrong throttle opening.
 */
/**
 * @brief Accelerator pedal demand. Full scale is Pedal::MAX (250).
 *
 * A scoped enum with a fixed underlying type is used as a STRONG typedef:
 *   - it is a distinct type, so a uint8_t will not implicitly convert either
 *     into or out of it,
 *   - comparisons between two pedal_pos_t work without ceremony,
 *   - arithmetic does NOT compile - unwrap with Pedal::raw_u8() first, which makes
 *     every unit conversion a deliberate, visible act,
 *   - it is exactly a uint8_t at the ABI level, so there is no runtime cost and
 *     nothing for the optimiser to remove.
 *
 * DO NOT use this type inside packed or wire structs (CAN frames, diag
 * payloads). Those describe a byte layout and must stay explicit uint8_t;
 * convert at the boundary with Pedal::raw_u8().
 */
enum class pedal_pos_t : uint8_t {};

// Locks the zero-overhead property: a scoped enum with a fixed underlying type
// must stay exactly its underlying type in size and ABI.
static_assert(sizeof(pedal_pos_t) == sizeof(uint8_t), "pedal_pos_t must stay register-sized");

namespace Pedal {
    /** Raw value corresponding to 100% pedal */
    static constexpr pedal_pos_t MAX = pedal_pos_t{250u};

    /** Fully released */
    static constexpr pedal_pos_t ZERO = pedal_pos_t{0u};

    /** Returned by EgsBaseCan::get_pedal_value() when there is no valid reading */
    static constexpr pedal_pos_t INVALID = pedal_pos_t{0xFFu};

    /** @brief Wrap a raw 0-250 reading. Name the unit at the boundary. */
    static constexpr pedal_pos_t from_raw(const uint8_t raw) {
        return pedal_pos_t{raw};
    }

    /**
     * @brief Unwrap to the underlying 0-250 value, for maths and wire structs.
     *
     * The width is in the NAME because it is invisible at the call site
     * otherwise - a return type cannot be seen from where it is used. Matches
     * the existing TCU_ROUND_TO_U8_SAT / TCU_ROUND_TO_I16_SAT convention.
     *
     * Prefer this over a plain (uint8_t) cast: a C-style cast compiles on
     * anything, so it would silently accept an RPM or a torque if the code
     * around it is later changed. This only accepts a pedal_pos_t.
     */
    static constexpr uint8_t raw_u8(const pedal_pos_t pedal) {
        return static_cast<uint8_t>(pedal);
    }

    /** @brief True if this is a real reading rather than the no-data sentinel */
    static constexpr bool is_valid(const pedal_pos_t pedal) {
        return pedal != INVALID;
    }

    /** @brief Pedal position -> 0..100 percent */
    static constexpr float to_percent(const pedal_pos_t pedal) {
        return ((float)raw_u8(pedal) * 100.0f) / (float)raw_u8(MAX);
    }

    /**
     * @brief Percentage -> pedal position, for expressing thresholds.
     *
     * Use this for constants (Pedal::percent(6.0f) rather than a bare 15) so a
     * threshold reads as the intent instead of the encoding.
     */
    static constexpr pedal_pos_t percent(const float pct) {
        return pedal_pos_t{(uint8_t)((pct * (float)raw_u8(MAX)) / 100.0f)};
    }
}

/**
 * @brief A temperature in whole degrees Celsius.
 *
 * Stored as plain Celsius everywhere inside the TCU. The reason this needs a
 * type is that it is carried over the wire in THREE different encodings, and
 * they are trivially easy to transpose:
 *   - engine temperatures arriving on CAN are unsigned with a -40 offset,
 *   - gearbox temperature sent on CAN is unsigned with a +50 offset,
 *   - the diagnostic payload uses a +40 offset.
 *
 * INT16_MAX is the "no reading" sentinel throughout - see Temp::INVALID.
 *
 * DO NOT use this type inside packed or wire structs; convert at the boundary.
 */
enum class temp_c_t : int16_t {};

static_assert(sizeof(temp_c_t) == sizeof(int16_t), "temp_c_t must stay register-sized");

namespace Temp {
    /** No valid reading. Matches the INT16_MAX sentinel used across the TCU. */
    static constexpr temp_c_t INVALID = temp_c_t{INT16_MAX};

    /** @brief Wrap a value that is already in whole degrees Celsius */
    static constexpr temp_c_t from_celsius(const int16_t celsius) {
        return temp_c_t{celsius};
    }

    /** @brief Unwrap to whole degrees Celsius, for maths and wire structs */
    static constexpr int16_t celsius_i16(const temp_c_t temp) {
        return static_cast<int16_t>(temp);
    }

    /** @brief True if this is a real reading rather than the no-data sentinel */
    static constexpr bool is_valid(const temp_c_t temp) {
        return temp != INVALID;
    }

    /**
     * @brief Decode an engine temperature from CAN (unsigned, -40 offset).
     *
     * Used for coolant, oil and intake air on EGS51/52/53 and CustomCAN. The
     * caller must reject the UINT8_MAX sentinel BEFORE calling this - 0xFF is a
     * valid-looking 215C once the offset is applied.
     */
    static constexpr temp_c_t from_can_u8_offset40(const uint8_t raw) {
        return temp_c_t{(int16_t)((int16_t)raw - 40)};
    }

    /**
     * @brief Origin of the Mercedes calibration temperature axis, in Celsius.
     *
     * The EGS calibration data (HYDR_PTR->pcs_map_y, mpc_flush_temp_threshold,
     * atf_density_minus_50c, ...) is not indexed in Celsius - its zero is -50C.
     * Anything looked up against factory calibration therefore needs shifting by
     * this offset first; anything looked up against a header this firmware
     * declares itself (fill_t_x_headers, pwm_tcc_y_headers) does NOT.
     */
    static constexpr int16_t CAL_AXIS_OFFSET_C = 50;

    /**
     * @brief Celsius -> the EGS calibration axis (-50C = 0), for map lookups.
     *
     * Use this instead of a bare "+ 50.0f" so that a lookup carries which axis
     * it is on. Mixing the two conventions silently reads the wrong map cell.
     */
    static constexpr float to_cal_axis(const temp_c_t temp) {
        return (float)(celsius_i16(temp) + CAL_AXIS_OFFSET_C);
    }

    /**
     * @brief Encode gearbox temperature for CAN (unsigned, +50 offset).
     *
     * Saturates at both ends. The open-coded form of this used to clamp only
     * the bottom, so anything above 205C wrapped around the byte.
     */
    static constexpr uint8_t to_can_u8_offset50(const temp_c_t temp) {
        return (uint8_t)((celsius_i16(temp) < -CAL_AXIS_OFFSET_C) ? 0
            : ((celsius_i16(temp) > (255 - CAL_AXIS_OFFSET_C)) ? 255
                : (celsius_i16(temp) + CAL_AXIS_OFFSET_C)));
    }
}

/**
 * @brief An engine or gearbox torque in whole Newton metres.
 *
 * Stored as plain signed Nm everywhere inside the TCU. Torque is negative on
 * overrun, which is why this is signed and why the CAN encoding carries a
 * -500 Nm zero point rather than being a plain unsigned quantity.
 *
 * Mercedes CAN carries torque as an unsigned field encoded (Nm + 500) * 4 -
 * 0.25 Nm per bit with raw 0 meaning -500 Nm. Every ECU flavour uses the same
 * encoding but a different field width, so the encode helper takes the width.
 *
 * INT16_MAX is the "no reading" sentinel - see Torque::INVALID.
 *
 * DO NOT use this type inside packed or wire structs; convert at the boundary.
 */
enum class torque_nm_t : int16_t {};

static_assert(sizeof(torque_nm_t) == sizeof(int16_t), "torque_nm_t must stay register-sized");

namespace Torque {
    /** No valid reading. Matches the INT16_MAX sentinel used across the TCU. */
    static constexpr torque_nm_t INVALID = torque_nm_t{INT16_MAX};

    /** No torque */
    static constexpr torque_nm_t ZERO = torque_nm_t{0};

    /** Raw 0 on the bus means this many Nm of negative (overrun) torque */
    static constexpr int16_t CAN_OFFSET_NM = 500;

    /** Bus resolution: 4 counts per Nm, i.e. 0.25 Nm per bit */
    static constexpr int16_t CAN_COUNTS_PER_NM = 4;

    /** @brief Wrap a value that is already in whole Nm */
    static constexpr torque_nm_t from_nm(const int16_t nm) {
        return torque_nm_t{nm};
    }

    /** @brief Unwrap to whole Nm, for maths and wire structs */
    static constexpr int16_t nm_i16(const torque_nm_t torque) {
        return static_cast<int16_t>(torque);
    }

    /** @brief True if this is a real reading rather than the no-data sentinel */
    static constexpr bool is_valid(const torque_nm_t torque) {
        return torque != INVALID;
    }

    /**
     * @brief Decode a torque field from CAN.
     *
     * Replaces the open-coded "(raw / 4) - 500" that appeared at fifteen
     * separate decode sites across EGS51/52/53 and CustomCAN. The caller must
     * reject the field's own not-available sentinel BEFORE calling this.
     */
    static constexpr torque_nm_t from_can_raw(const int32_t raw) {
        return torque_nm_t{(int16_t)((raw / CAN_COUNTS_PER_NM) - CAN_OFFSET_NM)};
    }

    /**
     * @brief Encode a torque request for CAN, saturating to the field width.
     *
     * @param nm      Requested torque. Also rejects NaN.
     * @param raw_max Largest value the destination bitfield can hold - 8191 for
     *                a 13 bit field such as GS218 M_EGS or ENG_RQ1_TCM
     *                EngTrq_Rq_TCM.
     *
     * Saturation is the point of this helper. Unclamped, an out of range
     * request wrapped inside the bitfield, so a large positive demand could
     * emerge at the engine as a large NEGATIVE one.
     */
    static constexpr uint16_t to_can_raw(const float nm, const uint16_t raw_max) {
        return (!((nm + (float)CAN_OFFSET_NM) * (float)CAN_COUNTS_PER_NM > 0.0f)) // also catches NaN
            ? 0u
            : (((nm + (float)CAN_OFFSET_NM) * (float)CAN_COUNTS_PER_NM > (float)raw_max)
                ? raw_max
                : (uint16_t)((nm + (float)CAN_OFFSET_NM) * (float)CAN_COUNTS_PER_NM));
    }

    /** Largest raw value a 13 bit torque field can carry (GS218, ENG_RQ1_TCM) */
    static constexpr uint16_t CAN_RAW_MAX_13BIT = 8191u;
}

/**
 * @brief A road wheel speed, carried at TWICE its real RPM.
 *
 * Shaft speeds (N2, N3, turbine, output, engine) are plain RPM and stay
 * uint16_t - they are all the same unit and a type buys nothing there. Wheel
 * speed is different: every Mercedes bus reports it at double the real RPM, and
 * that factor of two lived only in field names and comments. It has to be
 * undone before a wheel speed can be compared or combined with anything else,
 * and the undo sat forty lines away from the average that needed it.
 *
 * This type makes the conversion the only way out of the value.
 *
 * DO NOT use this type inside packed or wire structs; convert at the boundary.
 */
enum class wheel_rpm_2x_t : uint16_t {};

static_assert(sizeof(wheel_rpm_2x_t) == sizeof(uint16_t), "wheel_rpm_2x_t must stay register-sized");

namespace WheelSpeed {
    /** Returned by the get_*_wheel() accessors when there is no valid reading */
    static constexpr wheel_rpm_2x_t INVALID = wheel_rpm_2x_t{UINT16_MAX};

    /** Wheel stopped */
    static constexpr wheel_rpm_2x_t ZERO = wheel_rpm_2x_t{0u};

    /** @brief Wrap a bus reading, which is already at 2x the real RPM */
    static constexpr wheel_rpm_2x_t from_raw_2x(const uint16_t raw) {
        return wheel_rpm_2x_t{raw};
    }

    /**
     * @brief Unwrap WITHOUT undoing the doubling.
     *
     * Only for wire structs, traces and diagnostics that carry the raw 2x
     * value onward. If you are going to do arithmetic with the result, you
     * almost certainly want to_rpm_u16() instead.
     */
    static constexpr uint16_t raw_2x_u16(const wheel_rpm_2x_t speed) {
        return static_cast<uint16_t>(speed);
    }

    /** @brief True if this is a real reading rather than the no-data sentinel */
    static constexpr bool is_valid(const wheel_rpm_2x_t speed) {
        return speed != INVALID;
    }

    /** @brief Convert to the real wheel RPM */
    static constexpr uint16_t to_rpm_u16(const wheel_rpm_2x_t speed) {
        return (uint16_t)(raw_2x_u16(speed) / 2u);
    }

    /**
     * @brief Mean real RPM of two wheels on an axle.
     *
     * This is the "(left + right) >> 2" in Shifter::set_vehicle_speed() - a
     * shift by two, not one, because both inputs are doubled. Written out as a
     * bare shift it reads like a bug.
     */
    static constexpr uint16_t mean_rpm_u16(const wheel_rpm_2x_t left, const wheel_rpm_2x_t right) {
        return (uint16_t)(((uint32_t)raw_2x_u16(left) + (uint32_t)raw_2x_u16(right)) / 4u);
    }
}

#endif // TCU_SCALING_H
