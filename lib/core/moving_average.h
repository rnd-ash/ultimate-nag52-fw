#ifndef __MOVING_AVERAGE_H__
#define __MOVING_AVERAGE_H__

#include <stdint.h>
#include <type_traits>
#include "tcu_alloc.h"
#include "tcu_maths.h"
#include <string.h>

template <typename T> class MovingAverage {
public:
    MovingAverage(uint16_t sample_count_max, bool allocate_iram = false) {
        if (allocate_iram) {
            this->samples = (T*)TCU_IRAM_ALLOC(sample_count_max * sizeof(T));
        } else {
            this->samples = (T*)TCU_HEAP_ALLOC(sample_count_max * sizeof(T));
        }
        this->max_samples = sample_count_max;
        if (nullptr == this->samples) {
            abort();
        }
        this->reset();
    }
    void add_sample(T sample) {
        total -= samples[this->sample_idx];
        samples[this->sample_idx] = sample;
        total += sample;
        this->sample_idx = (this->sample_idx + 1) % this->max_samples;
        if (this->num_samples < this->max_samples) {
            this->num_samples++;
        }
    }
    T get_average() const {
        if (this->total == 0) {
            return 0;
        } else {
            return this->total / MIN(this->num_samples, this->max_samples);
        }
    }

    T front() const {
        return this->samples[sample_idx];
    }

    T back() const {
        if (this->sample_idx == 0) { // Safety for underflow
            if (this->num_samples < this->max_samples) { // Has not wrapped around, so back is pos 0
                return this->samples[0];
            } else {
                return this->samples[this->max_samples-1]; // Tail is the end of the buffer
            }
        } else {
            return this->samples[sample_idx-1];
        }
    }

    void reset() {
        memset(this->samples, 0x00, sizeof(T)*this->max_samples);
        this->total = 0;
        this->avg = 0;
        this->sample_idx = 0;
        this->num_samples = 0;
    }

    bool init_ok() const {
        return nullptr != this->samples;
    }

    bool has_full_samples() const {
        return this->num_samples == this->max_samples;
    }

    bool reset_done() const {
        return this->sample_idx == 0 && this->num_samples == 0;
    }

private:
    T* samples;
    T total;
    T avg;
    uint16_t sample_idx;
    uint16_t max_samples;
    uint16_t num_samples;
};

#endif