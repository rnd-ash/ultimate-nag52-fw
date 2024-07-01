#ifndef __FO_AVERAGE_H__
#define __FO_AVERAGE_H__

#include <stdint.h>
#include <type_traits>

/**
 * @brief First order average class (Low pass signal filter)
 * 
 * This filter works by keeping track of f(x) and f(x-1)
 * 
 * The average is calculated (With sample size 'k' using the following equation):
 * f(x) = (x + (f(x-1)*k)) / (k+1)
 * 
 * @tparam T The number type
 */
template <typename T> class FirstOrderAverage {
public:
    /**
     * Create a new first order average filter.
     * 
     * IMPORTANT. `samples` variable CAN NEVER be set to greater than 254! This will be
     * restricted in the constructor to max it out at 254.
     * 
     * Setting samples to 0 means no filtering (Output = Input)
    */
    FirstOrderAverage(uint8_t samples) {
        if (samples > 254) {
            // SAFETY
            this->sample_count = 254;
        } else {
            this->sample_count = samples;
        }
        this->reset();
    }

    void reset_with_start(T inital_value) {
        this->last_sample = inital_value*100;
        this->current_sample = inital_value*100;
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