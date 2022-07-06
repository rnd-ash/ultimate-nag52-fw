/** @file */
#ifndef __DRIVING_PROFILER_H__
#define __DRIVING_PROFILER_H__

#include "data.h"
#include "common_structs.h"

#define NUM_SAMPLES_PER_SEC 10
#define SAMPLE_INTERNVAL_MS 1000/NUM_SAMPLES_PER_SEC

template <typename T>
struct MovingAverageData {
    T array[NUM_SAMPLES_PER_SEC];
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
};

template<typename T> void add_to_moving_avg(MovingAverageData<T>* sample_data, T new_value) {
    sample_data->sum -= sample_data->array[sample_data->sample_id];
    sample_data->array[sample_data->sample_id] = new_value;
    sample_data->sum += new_value;
    sample_data->sample_id = (sample_data->sample_id+1) % NUM_SAMPLES_PER_SEC;
    sample_data->prev_avg = sample_data->curr_avg;
    sample_data->curr_avg = sample_data->sum / NUM_SAMPLES_PER_SEC;
} 

class DrivingProfile {
    public:
        DrivingProfile();
        void update(SensorData sensors);
        DrivingData* get_data();
    private:
        unsigned long last_update_ms = 0;
        DrivingData data;
        MovingAverageData<uint16_t> rpm_samples;
        MovingAverageData<int> speed_samples;
        MovingAverageData<int> pedal_samples;
};

#endif