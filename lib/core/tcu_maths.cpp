#include "tcu_maths.h"

float scale_number(float raw, float new_min, float new_max, float raw_min, float raw_max) {
    float raw_limited = MAX(raw_min, MIN(raw, raw_max));
    return (((new_max - new_min) * (raw_limited - raw_min)) / (raw_max - raw_min)) + new_min;
}


float progress_between_targets(float current, float start, float end) {
    return ((100.0 * (current - start)) / (end - start));
}