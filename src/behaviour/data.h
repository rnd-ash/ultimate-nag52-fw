#ifndef __BEHAVIOUR_DATA_H__
#define __BEHAVIOUR_DATA_H__

// Only affect the automatic drive profiles (Manual mode is unaffected)
enum class DriveStyle {
    // Lots of fast on / fast off with rapid braking
    Agressive,
    // Lots of fast acceleration
    Sporty,
    // Being purposely smooth
    Taxi,
    // Standard driving
    Normal,
    // Super slow acceleration, but late response for braking action
    Grandma,
};

typedef struct {
    // Current speed in KMH
    int current_vehicle_speed_kmh;
    // Pedal acceleration (%/sec)
    int pedal_acceleration;
    // Vehicle acceleration (KMH/sec^2)
    int vehicle_acceleration;
    // Engine acceleration (RPM/sec^2)
    int engine_acceleration;
    // Is the brake pedal pressed?
    bool is_braking;
    // Incline estimate (-10 = big downhill, 0 = level, 10 = big uphill)
    int incline_estimate;
    // Towing mode
    bool is_towing;
    // Time that vehicle speed hasn't changed much (Cruising, in ms)
    int constant_speed_time;

    // Based on current deceleration of the car, how many seconds until we stop?
    // If INT32_MAX is returned, that is because we are accelerating (Not possible to stop)
    int num_seconds_to_stop() {
        if (vehicle_acceleration >= 0) {
            return __INT32_MAX__;
        } else {
            return (current_vehicle_speed_kmh/vehicle_acceleration) * -1;
        }
    }
} DrivingData;

#endif