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

typedef struct {
    float new_min;
    float new_max;
    float raw_min;
    float raw_max;
} __attribute__ ((packed)) LinearInterpSetting;

float linear_interp(float start_value, float end_value, uint16_t current_elapsed, uint16_t interp_duration);

float scale_number(float raw, float new_min, float new_max, float raw_min, float raw_max);
float scale_number(float raw, LinearInterpSetting* settings);

float progress_between_targets(const float current, const float start, const float end);

/// @brief Calulates interpolated value between given values of function f_1 and f_2 for given value x.
/// @param f_1 the first function value
/// @param f_2 the second function value
/// @param x_1 the first x-value
/// @param x_2 the second x-value
/// @param x the x-value to calculate the interpolated function value for
/// @return the interpolated function value
float interpolate(const float f_1, const float f_2, const int16_t x_1, const int16_t x_2, const float x);

/// @brief Searches for a given value in values. idvalue_min and idvalue_max will be set to the indices between the value is found. This fuction assumes, that values has an ascending order.
/// @param value The value to be searched for.
/// @param values The array with the values to be searched within.
/// @param size The size of values.
/// @param idvalue_min The index for the value smaller than the value to be searched for.
/// @param idvalue_max The index for the value greater than the value to be searched for.
void search_value(const int16_t value, int16_t *values, const uint16_t size, uint16_t *idvalue_min, uint16_t *idvalue_max);

#endif // TCU_MATHS_H