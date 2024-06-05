#include "driving_profiler.h"
#include "clock.hpp"

void DrivingProfile::update(SensorData sensors) {
    if (GET_CLOCK_TIME() - this->last_update_ms < SAMPLE_INTERNVAL_MS) {
        return;
    }
    add_to_moving_avg(&this->rpm_samples, sensors.engine_rpm);
    this->last_update_ms = GET_CLOCK_TIME();
}