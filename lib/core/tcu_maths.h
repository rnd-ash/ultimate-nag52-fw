#ifndef TCU_MATHS_H
#define TCU_MATHS_H

#include <stdint.h>

// Core maths and calculation stuff this the TCM uses

#ifndef MAX
    #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
    #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

float scale_number(float raw, float new_min, float new_max, float raw_min, float raw_max);
float progress_between_targets(float current, float start, float end);

/// @brief Calulates interpolated value between given values of function f_1 and f_2 for given value x.
/// @param f_1 the first function value
/// @param f_2 the second function value
/// @param x_1 the first x-value
/// @param x_2 the second x-value
/// @param x the x-value to calculate the interpolated function value for
/// @return the interpolated function value
float interpolate(const float f_1, const float f_2, const int16_t x_1, const int16_t x_2, const float x);

#endif // TCU_MATHS_H