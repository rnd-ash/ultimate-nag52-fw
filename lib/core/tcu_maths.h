#ifndef TCU_MATHS_H
#define TCU_MATHS_H

#include <stdint.h>

// Core maths and calculation stuff this the TCM uses

#ifdef __cplusplus
namespace tcu_math_detail {
    template <typename A, typename B>
    constexpr auto max_once(A a, B b) {
        return (a < b) ? b : a;
    }

    template <typename A, typename B>
    constexpr auto min_once(A a, B b) {
        return (b < a) ? b : a;
    }
}
#endif

#ifndef MAX
    #ifdef __cplusplus
        #define MAX(a, b) (tcu_math_detail::max_once((a), (b)))
    #else
        #define MAX(a, b) (((a) > (b)) ? (a) : (b))
    #endif
#endif

#ifndef MIN
    #ifdef __cplusplus
        #define MIN(a, b) (tcu_math_detail::min_once((a), (b)))
    #else
        #define MIN(a, b) (((a) < (b)) ? (a) : (b))
    #endif
#endif

typedef struct {
    float in;
    float out;
} interp_mapping_t;

#define RAMP_FUNC_INTERP_POINTS (21)

const static interp_mapping_t EASE_IN_EASE_OUT_INTERP_LOOKUP[RAMP_FUNC_INTERP_POINTS] = {
    {0.0f,   0.00f},
    {5.0f,   0.61f},
    {10.0f,  2.45f},
    {15.0f,  5.45f},
    {20.0f,  9.55f},
    {25.0f, 14.65f},
    {30.0f, 20.61f},
    {35.0f, 27.30f},
    {40.0f, 34.55f},
    {45.0f, 42.17f},
    {50.0f, 50.00f},
    {55.0f, 57.82f},
    {60.0f, 65.45f},
    {65.0f, 72.70f},
    {70.0f, 79.39f},
    {75.0f, 85.82f},
    {80.0f, 90.45f},
    {85.0f, 94.55f},
    {90.0f, 97.55f},
    {95.0f, 99.38f},
    {100.0f, 100.00f},
};

const static interp_mapping_t EASE_IN_INTERP_LOOKUP[RAMP_FUNC_INTERP_POINTS] = {
    {0.0f,   0.00f},
    {5.0f,   0.30f},
    {10.0f,  1.23f},
    {15.0f,  2.76f},
    {20.0f,  4.89f},
    {25.0f,  7.61f},
    {30.0f, 10.89f},
    {35.0f, 14.74f},
    {40.0f, 19.10f},
    {45.0f, 23.96f},
    {50.0f, 29.29f},
    {55.0f, 35.06f},
    {60.0f, 41.22f},
    {65.0f, 47.75f},
    {70.0f, 54.60f},
    {75.0f, 61.73f},
    {80.0f, 69.10f},
    {85.0f, 76.66f},
    {90.0f, 84.36f},
    {95.0f, 92.15f},
    {100.0f, 100.00f},
};

const static interp_mapping_t EASE_OUT_INTERP_LOOKUP[RAMP_FUNC_INTERP_POINTS] = {
    {0.0f,   0.00f},
    {5.0f,   7.85f},
    {10.0f, 15.64f},
    {15.0f, 23.35f},
    {20.0f, 30.90f},
    {25.0f, 38.27f},
    {30.0f, 45.40f},
    {35.0f, 52.25f},
    {40.0f, 58.78f},
    {45.0f, 64.95f},
    {50.0f, 70.71f},
    {55.0f, 76.04f},
    {60.0f, 80.90f},
    {65.0f, 85.26f},
    {70.0f, 89.10f},
    {75.0f, 92.39f},
    {80.0f, 95.12f},
    {85.0f, 97.24f},
    {90.0f, 98.77f},
    {95.0f, 99.69f},
    {100.0f, 100.00f},
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


float interpolate_float(float raw, const LinearInterpSetting* settings, InterpType interp_type);

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

int linear_ramp_with_timer(int start, int end, int current_timer_val);
short linear_interp_with_percentage(uint16_t percentage, short new_value, short last_filtered_val);

int32_t first_order_filter(uint8_t sample_count, int32_t new_val, int32_t last_val);
float first_order_filter_f(uint8_t sample_count, int32_t new_val, float last_val);

#endif // TCU_MATHS_H