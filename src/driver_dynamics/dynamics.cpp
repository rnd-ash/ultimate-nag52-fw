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
        this->tracked_delta = first_order_filter(25, delta*100, this->tracked_delta);
    }
}

void DeltaTracker::reset() {
    this->first_val = true;
    this->tracked_delta = 0;
}

int32_t DeltaTracker::get_delta() {
    return this->tracked_delta/100;
}