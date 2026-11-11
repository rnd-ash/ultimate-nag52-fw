#include "egs51_clutch_speed.h"
#include "clutch_speed.hpp"
#include "tcu_io/tcu_io.hpp"

// This equation is used multiple times when calculating the speed of a clutch that goes from stationary to moving with the input shaft speed
//
// r_low (The lower ratio of the target and actual gear)
// r_high (The higher ratio of the target and actual gear)
// Inverse - Only used in Reverse gear as rear sun gear spins in reverse due to B3 being applied
int16_t get_speed_long_eq(const uint16_t output_speed, const uint16_t input, const float r_low, const float r_high, bool invert = false) {
    float num = 0;
    if (invert) {
        num = r_high * ((r_low * (float)output_speed) + (float)input);
    } else {
        num = r_high * ((r_low * (float)output_speed) - (float)input);
    }
    return num / (r_low - r_high);
}

ShiftClutchData ClutchSpeedModel::get_shifting_clutch_speeds(const SpeedSensors speeds, const GearChange req, const GearRatioInfo* ratios) {
    const auto staged = Egs51ClutchSpeed::calculate(static_cast<unsigned>(req),
        speeds.n2, speeds.n3, speeds.turbine, speeds.output,
        Egs51ClutchSpeed::ratio_word(ratios[RAT_2_IDX].ratio),
        Egs51ClutchSpeed::ratio_word(ratios[RAT_3_IDX].ratio),
        Egs51ClutchSpeed::ratio_word(ratios[RAT_4_IDX].ratio));
    return {staged.applying, staged.releasing, staged.rear_sun};
}

ClutchSpeeds ClutchSpeedModel::get_clutch_speeds_debug(
    const SpeedSensors speeds,
    const GearboxGear last_motion_gear,
    const GearboxGear actual,
    const GearboxGear target,
    const GearRatioInfo* ratios
) {
    ClutchSpeeds cs = {0,0,0,0,0,0};
    // Neutral handling
    if (actual == GearboxGear::Neutral || target == GearboxGear::Neutral || actual == GearboxGear::Park || target == GearboxGear::Park) {
        GearboxGear t_gear = last_motion_gear;


        if ((actual == GearboxGear::Neutral && target != GearboxGear::Neutral) || (actual == GearboxGear::Park && target != GearboxGear::Park)) {
            t_gear = target;
        }
        if (t_gear == GearboxGear::First) {
            cs.k1 = (int16_t)speeds.n2 - (int16_t)speeds.n3;
            cs.b1 = 0;
            cs.b2 = speeds.turbine;
        } else if (t_gear == GearboxGear::Second) {
            cs.k1 = 0;
            cs.b1 = speeds.n3;
            cs.b2 = speeds.n3;
        } else if (t_gear == GearboxGear::Third) {
            cs.k1 = 0;
            cs.b1 = speeds.n3;
            cs.b2 = speeds.n3;
        } else if (t_gear == GearboxGear::Fourth) {
            cs.k1 = 0;
            cs.b1 = speeds.n3;
            cs.b2 = speeds.n3;
        } else if (t_gear == GearboxGear::Fifth) {
            cs.k1 = 0;
            cs.b1 = speeds.turbine;
            cs.b2 = get_speed_long_eq(speeds.output, speeds.turbine, ratios[RAT_3_IDX].ratio, ratios[RAT_4_IDX].ratio);
        } else if (t_gear == GearboxGear::Reverse_First) {
            cs.k1 = (int16_t)speeds.n2 - (int16_t)speeds.n3;
            cs.b1 = 0;
            cs.b2 = get_speed_long_eq(speeds.output, speeds.turbine, ratios[RAT_3_IDX].ratio, ratios[RAT_4_IDX].ratio, true);
        } else if (t_gear == GearboxGear::Reverse_Second) {
            cs.k1 = 0;
            cs.b1 = speeds.turbine;
            cs.b2 = get_speed_long_eq(speeds.output, speeds.turbine, ratios[RAT_3_IDX].ratio, ratios[RAT_4_IDX].ratio, true);
        }
    } else if (0 != speeds.output) {
        if (target == actual) {
            // Same gear
            if (actual == GearboxGear::First) {
                cs.k1 = (int16_t)speeds.n2 - (int16_t)speeds.n3;
                cs.k2 = (int16_t)speeds.turbine - (ratios[RAT_3_IDX].ratio * (int16_t)speeds.output);
                cs.k3 = 0;
                cs.b1 = 0;
                cs.b2 = 0;
                cs.b3 = ratios[RAT_3_IDX].ratio * (int16_t)speeds.output;
            } else if (actual == GearboxGear::Second) {
                cs.k1 = 0;
                cs.k2 = (int16_t)speeds.n3 - (ratios[RAT_3_IDX].ratio * (int16_t)speeds.output);
                cs.k3 = 0;
                cs.b1 = speeds.n3;
                cs.b2 = 0;
                cs.b3 = ratios[RAT_3_IDX].ratio * (int16_t)speeds.output;
            } else if (actual == GearboxGear::Third) {
                cs.k1 = 0;
                cs.k2 = 0;
                cs.k3 = (ratios[RAT_3_IDX].ratio*(ratios[RAT_2_IDX].ratio*(float)speeds.output - (float)speeds.n3)) / (ratios[RAT_2_IDX].ratio - ratios[RAT_3_IDX].ratio);
                cs.b1 = speeds.n3;
                cs.b2 = 0;
                cs.b3 = speeds.n3;
            } else if (actual == GearboxGear::Fourth) {
                cs.k1 = 0;
                cs.k2 = 0;
                cs.k3 = 0;
                cs.b1 = speeds.n3;
                cs.b2 = (ratios[RAT_3_IDX].ratio*(float)speeds.output - (float)speeds.n3)/(ratios[RAT_3_IDX].ratio - ratios[RAT_4_IDX].ratio);
                cs.b3 = speeds.n3;
            } else if (actual == GearboxGear::Fifth) {
                cs.b2 = (ratios[RAT_3_IDX].ratio*(float)speeds.output - (float)speeds.turbine)/(ratios[RAT_3_IDX].ratio - ratios[RAT_4_IDX].ratio);
                cs.k1 = (int16_t)speeds.n2 - (int16_t)speeds.n3;
                cs.k2 = 0;
                cs.k3 = 0;
                cs.b1 = 0;
                cs.b3 = speeds.turbine;
            }
            else if (actual == GearboxGear::Reverse_First) {
                cs.k1 = (int16_t)speeds.n2 - (int16_t)speeds.n3;
                cs.k2 = speeds.turbine;
                cs.k3 = 0;
                cs.b1 = 0;
                cs.b2 = get_speed_long_eq(speeds.output, speeds.turbine, ratios[RAT_3_IDX].ratio, ratios[RAT_4_IDX].ratio, true);
                cs.b3 = 0;
            } else if (actual == GearboxGear::Reverse_Second) {
                cs.k1 = 0;
                cs.k2 = speeds.turbine;
                cs.k3 = 0;
                cs.b1 = speeds.turbine;
                cs.b2 = get_speed_long_eq(speeds.output, speeds.turbine, ratios[RAT_3_IDX].ratio, ratios[RAT_4_IDX].ratio, true);
                cs.b3 = 0;
            }
        } else {
            ShiftClutchData scd;
            if ((target == GearboxGear::First && actual == GearboxGear::Second) || (target == GearboxGear::Second && actual == GearboxGear::First)) { // 1-2 or 2-1
                scd = get_shifting_clutch_speeds(speeds, GearChange::_1_2, ratios);
                cs.k1 = scd.on_clutch_speed;
                cs.k2 = (int16_t)speeds.turbine - (ratios[RAT_3_IDX].ratio * (int16_t)speeds.output);
                cs.k3 = 0;
                cs.b1 = scd.off_clutch_speed;
                cs.b2 = 0;
                cs.b3 = ratios[RAT_3_IDX].ratio * (int16_t)speeds.output;
            } else if ((target == GearboxGear::Second && actual == GearboxGear::Third) || (target == GearboxGear::Third && actual == GearboxGear::Second)) { // 2-3 or 3-2
                scd = get_shifting_clutch_speeds(speeds, GearChange::_2_3, ratios);
                cs.k1 = 0;
                cs.k2 = scd.on_clutch_speed;
                cs.k3 = scd.off_clutch_speed;
                cs.b1 = speeds.n3;
                cs.b2 = 0;
                cs.b3 = ratios[RAT_3_IDX].ratio * (int16_t)speeds.output;
            } else if ((target == GearboxGear::Third && actual == GearboxGear::Fourth) || (target == GearboxGear::Fourth && actual == GearboxGear::Third)) { // 3-4 or 4-3
                scd = get_shifting_clutch_speeds(speeds, GearChange::_3_4, ratios);
                cs.k1 = 0;
                cs.k2 = 0;
                cs.k3 = scd.on_clutch_speed;
                cs.b1 = speeds.n3;
                cs.b2 = scd.off_clutch_speed;
                cs.b3 = speeds.n3;
            } else if ((target == GearboxGear::Fourth && actual == GearboxGear::Fifth) || (target == GearboxGear::Fifth && actual == GearboxGear::Fourth)) { // 4-5 or 5-4
                scd = get_shifting_clutch_speeds(speeds, GearChange::_4_5, ratios);
                cs.k1 = scd.off_clutch_speed;
                cs.k2 = 0;
                cs.k3 = 0;
                cs.b1 = scd.on_clutch_speed;
                cs.b2 = scd.rear_sun_speed;
                cs.b3 = speeds.turbine;
            }
        }
    }
    return cs;
}
