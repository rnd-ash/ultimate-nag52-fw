// Profile code

#ifndef __PROFILES_H_
#define __PROFILES_H_

#include "canbus/can_hal.h"
#include "common_structs.h"

#define MAX_PROFILES 4
typedef int16_t pressure_map[11];

/**
 * A profile is designed to read the current conditions of the gearbox and request the gearbox to do something
 */
class AbstractProfile {
public:
    virtual GearboxProfile get_profile() const;
    virtual char get_display_gear(GearboxGear target, GearboxGear actual);
    virtual bool should_upshift(GearboxGear current_gear, SensorData* sensors);
    virtual bool should_downshift(GearboxGear current_gear, SensorData* sensors);
    virtual ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction = nullptr) {
        switch (requested) {
            case ProfileGearChange::ONE_TWO: // 1-2
                return ShiftData { .spc_perc = 310, .mpc_perc = 310, .targ_ms = 500 };
            case ProfileGearChange::TWO_THREE: // 2-3
                return ShiftData { .spc_perc = 320, .mpc_perc = 320, .targ_ms = 500 };
            case ProfileGearChange::THREE_FOUR: // 3-4
                return ShiftData { .spc_perc = 320, .mpc_perc = 320, .targ_ms = 500 };
            case ProfileGearChange::FOUR_FIVE: // 4-5
                return ShiftData { .spc_perc = 320, .mpc_perc = 320, .targ_ms = 500 };
            case ProfileGearChange::FIVE_FOUR: // 5-4
                return ShiftData { .spc_perc = 250, .mpc_perc = 250, .targ_ms = 500 };
            case ProfileGearChange::FOUR_THREE: // 4-3
                return ShiftData { .spc_perc = 230, .mpc_perc = 230, .targ_ms = 500 };
            case ProfileGearChange::THREE_TWO: // 3-2
                return ShiftData { .spc_perc = 300, .mpc_perc = 300, .targ_ms = 500 };
            case ProfileGearChange::TWO_ONE: // 2-1
                return ShiftData { .spc_perc = 300, .mpc_perc = 300, .targ_ms = 500 };
            default: // WTF
                return ShiftData { .spc_perc = 300, .mpc_perc = 300, .targ_ms = 500 };
        }
        return ShiftData { .spc_perc = 300, .mpc_perc = 300, .targ_ms = 500 };
    }
    virtual GearboxGear get_start_gear() const {
        return GearboxGear::First;
    }
    void increment_subprofile() {
        profile_id += 1;
        if (profile_id >= MAX_PROFILES) {
            profile_id = 0;
        }
    }
protected:
    uint8_t profile_id = 0;
};

class AgilityProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Agility; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction = nullptr) override;
};

class ComfortProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Comfort; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction = nullptr) override;
    GearboxGear get_start_gear() const override {
        return GearboxGear::Second;
    }
};

class WinterProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Winter; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction = nullptr) override;
    GearboxGear get_start_gear() const override {
        return GearboxGear::Second;
    }
};

class StandardProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Standard; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction = nullptr) override;
    unsigned long last_shift_time = 0;
}; 

class ManualProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Manual; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
};

extern AgilityProfile* agility;
extern ComfortProfile* comfort;
extern WinterProfile* winter;
extern ManualProfile* manual;
extern StandardProfile* standard;

#endif