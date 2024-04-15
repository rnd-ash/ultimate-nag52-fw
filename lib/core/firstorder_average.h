#ifndef __FO_AVERAGE_H__
#define __FO_AVERAGE_H__

#include <stdint.h>
#include <type_traits>
#include "tcu_alloc.h"
#include "tcu_maths.h"
#include <string.h>

template <typename T> class FirstOrderAverage {
public:
    FirstOrderAverage(uint8_t samples) {
        if (samples > 254) {
            // SAFETY
            this->sample_count = 254;
        } else {
            this->sample_count = samples;
        }
        this->reset();
    }

    void add_sample(T sample) {
        this->last_sample = this->current_sample;
        this->current_sample = ((sample*100) + (this->sample_count*this->last_sample)) / (this->sample_count + 1);
    }
    T get_average() const {
        return this->current_sample/100;
    }

    float get_average_float() const {
        return (float)this->current_sample/100.0;
    }

    void reset() {
        this->last_sample = 0;
        this->current_sample = 0;
    }

private:
    T current_sample;
    T last_sample;
    uint8_t sample_count;
};

#endif