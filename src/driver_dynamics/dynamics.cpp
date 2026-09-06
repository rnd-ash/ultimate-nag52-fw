#include "dynamics.h"
#include "tcu_maths.h"

DeltaTracker::DeltaTracker(uint8_t samples) {
    this->samples = samples;
    this->last_value = 0;
    this->tracked_delta = 0;
    this->first_val = true;
}

void DeltaTracker::update(int32_t val) {
    if (first_val) {
        first_val = false;
    } else {
        int delta = val - this->last_value;
        // 'samples' is the constructor argument - it used to be stored and then
        // ignored in favour of a hardcoded 25.
        this->tracked_delta = first_order_filter(this->samples, delta*100, this->tracked_delta);
    }
    // MUST happen on every call, including the first. Without it last_value
    // stayed at 0 forever and 'delta' was really just the absolute value.
    this->last_value = val;
}

void DeltaTracker::reset() {
    this->first_val = true;
    this->tracked_delta = 0;
}

int32_t DeltaTracker::get_delta() const {
    return this->tracked_delta/100;
}