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
    float in;
    float out;
} interp_mapping_t;

#define RAMP_FUNC_INTERP_POINTS 21

const static interp_mapping_t EASE_IN_EASE_OUT_INTERP_LOOKUP[RAMP_FUNC_INTERP_POINTS] = {
    {0,   0.00},
    {5,   0.61},
    {10,  2.45},
    {15,  5.45},
    {20,  9.55},
    {25, 14.65},
    {30, 20.61},
    {35, 27.30},
    {40, 34.55},
    {45, 42.17},
    {50, 50.00},
    {55, 57.82},
    {60, 65.45},
    {65, 72.70},
    {70, 79.39},
    {75, 85.82},
    {80, 90.45},
    {85, 94.55},
    {90, 97.55},
    {95, 99.38},
    {100, 100.00},
};

const static interp_mapping_t EASE_IN_INTERP_LOOKUP[RAMP_FUNC_INTERP_POINTS] = {
    {0,   0.00},
    {5,   0.30},
    {10,  1.23},
    {15,  2.76},
    {20,  4.89},
    {25,  7.61},
    {30, 10.89},
    {35, 14.74},
    {40, 19.10},
    {45, 23.96},
    {50, 29.29},
    {55, 35.06},
    {60, 41.22},
    {65, 47.75},
    {70, 54.60},
    {75, 61.73},
    {80, 69.10},
    {85, 76.66},
    {90, 84.36},
    {95, 92.15},
    {100, 100.00},
};

const static interp_mapping_t EASE_OUT_INTERP_LOOKUP[RAMP_FUNC_INTERP_POINTS] = {
    {0,   0.00},
    {5,   7.85},
    {10, 15.64},
    {15, 23.35},
    {20, 30.90},
    {25, 38.27},
    {30, 45.40},
    {35, 52.25},
    {40, 58.78},
    {45, 64.95},
    {50, 70.71},
    {55, 76.04},
    {60, 80.90},
    {65, 85.26},
    {70, 89.10},
    {75, 92.39},
    {80, 95.12},
    {85, 97.24},
    {90, 98.77},
    {95, 99.69},
    {100, 100.00},
};

enum class InterpType: uint8_t {
    Linear = 0,
    EaseInEaseOut = 1,
    EaseIn = 2,
    EaseOut = 3,
};

typedef struct {
    float new_min;
    float new_max;
    float raw_min;
    float raw_max;
} __attribute__ ((packed)) LinearInterpSetting;


float scale_number(float raw, float new_min, float new_max, float raw_min, float raw_max);
float scale_number(float raw, const LinearInterpSetting* settings);
// Faster version of [scale_number], mainly used for ISRs where float cannot be used
int scale_number_int(int raw, int new_min, int new_max, int raw_min, int raw_max);
float interpolate_float(float raw, float new_min, float new_max, float raw_min, float raw_max, InterpType interp_type);

/**
 * @brief Perform linear interpolation on an integer. This is same as [interpolate_float], but uses ints. This is used 
 * in certain ISRs or areas where performing float operations is not allowed
 * @param raw Raw input value
 * @param new_min Scale target output minimum
 * @param new_max Scale target output maximum
 * @param raw_min Raw input value minimum value
 * @param raw_max Raw input value maximum value
 * @return Interpolated value
 */
int interpolate_int(int raw, int new_min, int new_max, int raw_min, int raw_max);

float interpolate_float(float raw, LinearInterpSetting* settings, InterpType interp_type);

float progress_between_targets(float current, float start, float end);
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
void search_value(const int16_t value, const int16_t *values, const uint16_t size, uint16_t *idvalue_min, uint16_t *idvalue_max);

#endif // TCU_MATHS_H