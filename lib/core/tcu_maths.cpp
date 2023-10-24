#include "tcu_maths.h"

float interpolate_float(float raw, float new_min, float new_max, float raw_min, float raw_max, InterpType interp_type) {
    float raw_limited = MAX(raw_min, MIN(raw, raw_max));
    // Function based interpolation
    const interp_mapping_t* ptr = nullptr;
    switch (interp_type) {
        case InterpType::EaseIn:
            ptr = EASE_IN_INTERP_LOOKUP;
            break;
        case InterpType::EaseInEaseOut:
            ptr = EASE_IN_EASE_OUT_INTERP_LOOKUP;
            break;
        case InterpType::EaseOut:
            ptr = EASE_OUT_INTERP_LOOKUP;
            break;
        default:
            break;
    }
    if (nullptr == ptr) {
        // linear
        return (((new_max - new_min) * (raw_limited - raw_min)) / (raw_max - raw_min)) + new_min;
    } else {
        // Function based interpolation
        float input_percentage = progress_between_targets(raw_limited, raw_min, raw_max);
        float input_percentage_as_int = (int16_t)input_percentage;
        float ret = new_min;
        if (input_percentage_as_int <= 0)
        {
            ret = new_min;
        }
        else if (input_percentage_as_int >= 100)
        {
            ret = new_max;
        }
        else
        {
            // We know interpolation maps have a step size of 5,
            // therefore, index for each point is very easy to calculate:
            // percentage / 5 = low_bound_idx
            uint8_t i = input_percentage_as_int/5;
            float percentage_output = interpolate(ptr[i].out, ptr[i+1].out, ptr[i].in, ptr[i+1].in, input_percentage);
            ret = (((new_max - new_min) * percentage_output) / 100.0) + new_min;
        }
        return ret;
    }
}

int interpolate_int(int raw, int new_min, int new_max, int raw_min, int raw_max) {
    int raw_limited = MAX(raw_min, MIN(raw, raw_max));
    return (((new_max - new_min) * (raw_limited - raw_min)) / (raw_max - raw_min)) + new_min;
}

float scale_number(float raw, const LinearInterpSetting* settings) {
    return scale_number(raw, settings->new_min, settings->new_max, settings->raw_min, settings->raw_max);
}

float interpolate_float(float raw, LinearInterpSetting* settings, InterpType interp_type) {
    return interpolate_float(raw, settings->new_min, settings->new_max, settings->raw_min, settings->raw_max, interp_type);
}

float interpolate(const float f_1, const float f_2, const int16_t x_1, const int16_t x_2, const float x)
{
    // cast values from signed integer values to floating values in order to avoid casting the same value twice
    const float x_1_f = (float)x_1;
    const float x_2_f = (float)x_2;
    // See https://en.wikipedia.org/wiki/Linear_interpolation for details. Return f_1, if x_1 and x_2 are identical.
    return (x_1 != x_2) ? f_1 + ((f_2 - f_1) / (x_2_f - x_1_f)) * (x - x_1_f) : f_1;
}


float progress_between_targets(const float current, const float start, const float end) {
    return ((100.0F * (current - start)) / (end - start));
}

void search_value(const int16_t value, const int16_t *values, const uint16_t size, uint16_t *idvalue_min, uint16_t *idvalue_max)
{
	// Set minimum index to the first element of the field.
    *idvalue_min = 0u;
    // Set maximum index to the last element of the field.
    *idvalue_max = size - 1u;
    // Check, if search value is smaller than smallest element of the field.
    if (value > values[0]) {
        if (value < values[*idvalue_max]) {
            // Search value is in between the limits of the smallest and the biggest element of the field.
            do {
                // Calculate the middle of the remaining list. If the size is odd, it is rounded down.
                uint16_t idvalue_mid = (*idvalue_min + *idvalue_max) >> 1;
                if (value < values[idvalue_mid]) {
                    // Search value is smaller than the element in the middle of the remaining list.
                    *idvalue_max = idvalue_mid;
                }
                else if (value > values[idvalue_mid]) {
                    // Search value is bigger than the element in the middle of the remaining list.
                    *idvalue_min = idvalue_mid;
                }
                else {
                    // Search value is also an element of the field.
                    *idvalue_min = idvalue_mid;
                    *idvalue_max = idvalue_mid;
                }
                // Reduce the remaining search area until it is narrowed down to two consecutive elements.
            } while (1u < ((*idvalue_max) - (*idvalue_min)));
        }
        else {
            // Search value is as big as or bigger then the biggest element in the field.
            *idvalue_min = *idvalue_max;
        }
    }
    else {
        // Search value is as small as or smaller than smallest element of the field.
        *idvalue_max = *idvalue_min;
    }
}
