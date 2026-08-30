#include "programselector.hpp"

GearboxProfile selectableProfileToProfile(SelectableGearboxProfile p) {
    GearboxProfile ret = GearboxProfile::Standard;
    switch (p) {
        case SelectableGearboxProfile::Standard:
            ret = GearboxProfile::Standard;
            break;
        case SelectableGearboxProfile::Comfort:
            ret = GearboxProfile::Comfort;
            break;
        case SelectableGearboxProfile::Agility:
            ret = GearboxProfile::Agility;
            break;
        case SelectableGearboxProfile::Manual:
            ret = GearboxProfile::Manual;
            break;
        case SelectableGearboxProfile::Race:
            ret = GearboxProfile::Race;
            break;
    }
    return ret;
}