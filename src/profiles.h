// Profile code

#ifndef __PROFILES_H_
#define __PROFILES_H_

#include "canbus/can_hal.h"

/**
 * A profile is designed to read the current conditions of the gearbox and request the gearbox to do something
 */
class AbstractProfile {
public:
    virtual GearboxProfile get_profile() const;
    virtual char get_display_gear(GearboxGear target, GearboxGear actual);
    virtual bool should_upshift(GearboxGear current_gear);
    virtual bool should_downshift(GearboxGear current_gear);
};

class AgilityProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Agility; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear);
    bool should_downshift(GearboxGear current_gear);
};

class ComfortProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Comfort; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear);
    bool should_downshift(GearboxGear current_gear);
};

class WinterProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Winter; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear);
    bool should_downshift(GearboxGear current_gear);
};

class StandardProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Standard; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear);
    bool should_downshift(GearboxGear current_gear);
};

class ManualProfile : public AbstractProfile {
public:
    GearboxProfile get_profile() const override { return GearboxProfile::Manual; }
    char get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear);
    bool should_downshift(GearboxGear current_gear);
};

#endif