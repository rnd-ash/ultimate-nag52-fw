#ifndef TCM_MATHS_H
#define TCM_MATHS_H

// Core maths and calculation stuff this the TCM uses

#ifndef MAX
    #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
    #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

float scale_number(float raw, float new_min, float new_max, float raw_min, float raw_max);

#endif // TCM_MATHS_H