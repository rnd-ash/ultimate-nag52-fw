#include "tcu_maths.h"

float interpolate_float(float raw, float new_min, float new_max, float raw_min, float raw_max) {
    float raw_limited = MAX(raw_min, MIN(raw, raw_max));
    return (((new_max - new_min) * (raw_limited - raw_min)) / (raw_max - raw_min)) + new_min;
}

int interpolate_int(int raw, int new_min, int new_max, int raw_min, int raw_max) {
    int raw_limited = MAX(raw_min, MIN(raw, raw_max));
    return (((new_max - new_min) * (raw_limited - raw_min)) / (raw_max - raw_min)) + new_min;
}

float interpolate_float(float raw, LinearInterpSetting* settings) {
    return interpolate_float(raw, settings->new_min, settings->new_max, settings->raw_min, settings->raw_max);
}

float interpolate(const float f_1, const float f_2, const int16_t x_1, const int16_t x_2, const float x)
{
    // cast values from signed integer values to floating values in order to avoid casting the same value twice
    const float x_1_f = (float)x_1;
    const float x_2_f = (float)x_2;
    // See https://en.wikipedia.org/wiki/Linear_interpolation for details. Return f_1, if x_1 and x_2 are identical.
    return (x_1 != x_2) ? f_1 + ((f_2 - f_1) / (x_2_f - x_1_f)) * (x - x_1_f) : f_1;
}


float progress_between_targets(float current, float start, float end) {
    return ((100.0F * (current - start)) / (end - start));
}