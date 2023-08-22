#ifndef __MOVING_AVERAGE_H__
#define __MOVING_AVERAGE_H__

#include <stdint.h>
#include <type_traits>
#include "tcu_alloc.h"
#include "tcu_maths.h"
#include <string.h>

class MovingAverage {
public:
    MovingAverage(uint8_t sample_count_max, bool allocate_iram = false) {
        if (allocate_iram) {
            this->samples = (int32_t*)TCU_IRAM_ALLOC(sample_count_max * sizeof(int32_t));
        } else {
            this->samples = (int32_t*)TCU_HEAP_ALLOC(sample_count_max * sizeof(int32_t));
        }
        this->max_samples = sample_count_max;
        this->reset();
    }
    void add_sample(int32_t sample) {
        total -= samples[this->sample_idx];
        samples[this->sample_idx] = sample;
        total += sample;
        this->sample_idx = (this->sample_idx + 1) % this->max_samples;
        if (this->num_samples < this->max_samples) {
            this->num_samples++;
        }
    }
    int32_t get_average() {
        if (this->total == 0) {
            return 0;
        } else {
            return this->total / MIN(this->num_samples, this->max_samples);
        }
    }

    void reset() {
        memset(this->samples, 0x00, sizeof(int32_t)*this->max_samples);
        this->total = 0;
        this->avg = 0;
        this->sample_idx = 0;
        this->num_samples = 0;
    }

    bool init_ok() {
        return nullptr != this->samples;
    }

    bool has_full_samples() {
        return this->num_samples == this->max_samples;
    }

    bool reset_done() {
        return this->sample_idx == 0 && this->num_samples == 0;
    }

private:
    int32_t* samples;
    int32_t total;
    int32_t avg;
    uint16_t sample_idx;
    uint16_t max_samples;
    uint16_t num_samples;
};

class MovingUnsignedAverage {
public:
    MovingUnsignedAverage(uint8_t sample_count_max, bool allocate_iram = false) {
        if (allocate_iram) {
            this->samples = (uint32_t*)TCU_IRAM_ALLOC(sample_count_max * sizeof(uint32_t));
        } else {
            this->samples = (uint32_t*)TCU_HEAP_ALLOC(sample_count_max * sizeof(uint32_t));
        }
        this->max_samples = sample_count_max;
        this->reset();
    }
    void add_sample(uint32_t sample) {
        total -= samples[this->sample_idx];
        samples[this->sample_idx] = sample;
        total += sample;
        this->sample_idx = (this->sample_idx + 1) % this->max_samples;
        if (this->num_samples < this->max_samples) {
            this->num_samples++;
        }
    }
    uint32_t get_average() {
        if (this->total == 0) {
            return 0;
        } else {
            return this->total / MIN(this->num_samples, this->max_samples);
        }
    }

    void reset() {
        memset(this->samples, 0x00, sizeof(uint32_t)*this->max_samples);
        this->total = 0;
        this->avg = 0;
        this->sample_idx = 0;
        this->num_samples = 0;
    }

    bool init_ok() {
        return nullptr != this->samples;
    }

    bool has_full_samples() {
        return this->num_samples == this->max_samples;
    }

    bool reset_done() {
        return this->sample_idx == 0 && this->num_samples == 0;
    }

private:
    uint32_t* samples;
    uint32_t total;
    uint32_t avg;
    uint16_t sample_idx;
    uint16_t max_samples;
    uint16_t num_samples;
};

class MovingFloatAverage {
public:
    MovingFloatAverage(uint8_t sample_count_max) {
        this->samples = (float*)TCU_HEAP_ALLOC(sample_count_max * sizeof(float));
        this->max_samples = sample_count_max;
        this->reset();
    }
    void add_sample(float sample) {
        total -= samples[this->sample_idx];
        samples[this->sample_idx] = sample;
        total += sample;
        this->sample_idx = (this->sample_idx + 1) % this->max_samples;
        if (this->num_samples < this->max_samples) {
            this->num_samples++;
        }
    }
    float get_average() {
        return this->total / (float)(MIN(this->num_samples, this->max_samples));
    }

    void reset() {
        memset(this->samples, 0x00, sizeof(float)*this->max_samples);
        this->total = 0;
        this->sample_idx = 0;
        this->num_samples = 0;
    }

    bool init_ok() {
        return nullptr != this->samples;
    }

    bool has_full_samples() {
        return this->num_samples == this->max_samples;
    }

    bool reset_done() {
        return this->sample_idx == 0 && this->num_samples == 0;
    }
private:
    float* samples;
    float total;
    uint16_t sample_idx;
    uint16_t max_samples;
    uint16_t num_samples;
};

#endif