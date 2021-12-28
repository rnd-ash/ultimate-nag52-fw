// Profile code

#ifndef __PROFILES_H_
#define __PROFILES_H_

#include "canbus/can_hal.h"
#include "common_structs.h"

#define MAX_PROFILES 4

float find_temp_multiplier(int temp_raw); // Todo move this function
uint16_t find_mpc_pressure(pressure_map map, SensorData* sensors, float shift_firmness = 1.0);
extern pressure_map map_1_2;
extern pressure_map map_2_3;
extern pressure_map map_3_4;
extern pressure_map map_4_5;

extern pressure_map map_2_1;
extern pressure_map map_3_2;
extern pressure_map map_4_3;
extern pressure_map map_5_4;

/**
 * A profile is designed to read the current conditions of the gearbox and request the gearbox to do something
 */
class AbstractProfile {
public:
    virtual GearboxProfile get_profile() const;
    virtual char get_display_gear(GearboxGear target, GearboxGear actual);
    virtual bool should_upshift(GearboxGear current_gear, SensorData* sensors);
    virtual bool should_downshift(GearboxGear current_gear, SensorData* sensors);
    virtual ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, float shift_speed = 1.0, float shift_firmness = 1.0) {
        return DEFAULT_SHIFT_DATA;
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
    ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, float shift_speed = 1.0, float shift_firmness = 1.0) override;
};

class ComfortProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Comfort; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, float shift_speed = 1.0, float shift_firmness = 1.0) override;
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
    ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, float shift_speed = 1.0, float shift_firmness = 1.0) override;
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
    void on_upshift_complete(ShiftResponse response, uint8_t from_gear, SensorData* sensors);
    ShiftData get_shift_data(ProfileGearChange requested, SensorData* sensors, float shift_speed = 1.0, float shift_firmness = 1.0) override;
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