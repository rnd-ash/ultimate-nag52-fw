#ifndef FO_AVERAGE_H
#define FO_AVERAGE_H

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
class FirstOrderAverage {
public:
    /**
     * Create a new first order average filter.
     * 
     * IMPORTANT. `samples` variable CAN NEVER be set to greater than 254! This will be
     * restricted in the constructor to max it out at 254.
     * 
     * Setting samples to 0 means no filtering (Output = Input)
    */
    explicit FirstOrderAverage(uint8_t samples, int32_t reset_value = 0) {
        if (samples > 254u) {
            // SAFETY
            this->sample_count = 254u;
        } else {
            this->sample_count = samples;
        }
        this->reset(reset_value);
    }

    void add_sample(int32_t sample) {
        last_sample = current_sample;
        current_sample = ((sample*100) + ((int32_t)sample_count*last_sample)) / ((int32_t)sample_count + 1);
    }

    int32_t get_average(void) const {
        return this->current_sample/100;
    }

    float get_average_float(void) const {
        return (float)this->current_sample/100;
    }

    void reset(int32_t reset_value = 0) {
        this->last_sample = reset_value*100;
        this->current_sample = reset_value*100;
    }

private:
    int32_t current_sample;
    int32_t last_sample;
    uint8_t sample_count;
};

#endif