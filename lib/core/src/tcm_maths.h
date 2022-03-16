#ifndef __TCM_MATHS_H__
#define __TCM_MATHS_H__

// Core maths and calculation stuff this the TCM uses

#ifndef MAX
    #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
    #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/*

template <typename T, int MAP_SIZE>
struct TcmMap {
    T map[MAP_SIZE];

    T get_value(T min, T actual, T max) {
        if (actual <= min) {
            return map[0];
        } else if (actual >= max) {
            return map[MAP_SIZE-1];
        } else {
            // Linear interpolate
            float dy = rpm_normalizer[max] - rpm_normalizer[min];
            float dx = max-min;
            return (rpm_normalizer[min] + ((dy/dx)) * (engine_rpm-(min*1000)));
        }
    }
};

template <typename T, int size>
struct MovingAverageData {
    T array[size];
    uint8_t sample_id;
    T sum;
    T total;
    T prev_avg;
    T curr_avg;

    MovingAverageData() {
        sum = 0;
        total = 0;
        sample_id = 0;
        prev_avg = 0;
        curr_avg = 0;
        memset(array, 0x00, sizeof(array));
    }
    void add_to_avg();
    T get_avg();
};

*/

#endif // __TCM_MATHS_H__