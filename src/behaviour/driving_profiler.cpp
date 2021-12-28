#include "driving_profiler.h"

void DrivingProfile::update(SensorData sensors) {
    if (sensors.current_timestamp_ms - this->last_update_ms < SAMPLE_INTERNVAL_MS) {
        return;
    }
    add_to_moving_avg(&this->rpm_samples, sensors.engine_rpm);
    this->last_update_ms = sensors.current_timestamp_ms;
}