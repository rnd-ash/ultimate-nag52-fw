// Profile code

#ifndef __PROFILES_H_
#define __PROFILES_H_

#include "canbus/can_hal.h"

/**
 * A profile is designed to read the current conditions of the gearbox and request the gearbox to do something
 */
class AbstractProfile {
public:
    const virtual GearboxProfile get_profile();
};

class AgilityProfile : public AbstractProfile {
public:
    const GearboxProfile get_profile() override { return GearboxProfile::Agility; }
};

class ComfortProfile : public AbstractProfile {
public:
    const GearboxProfile get_profile() override { return GearboxProfile::Comfort; }
};

class WinterProfile : public AbstractProfile {
public:
    const GearboxProfile get_profile() override { return GearboxProfile::Winter; }
};

class StandardProfile : public AbstractProfile {
public:
    const GearboxProfile get_profile() override { return GearboxProfile::Standard; }
};

class ManualProfile : public AbstractProfile {
public:
    const GearboxProfile get_profile() override { return GearboxProfile::Manual; }
};

#endif