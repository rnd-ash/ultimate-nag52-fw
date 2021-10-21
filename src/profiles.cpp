#include "profiles.h"

char AgilityProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
   switch (target) {
        case GearboxGear::Park:
            return 'P';
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return 'R';
        case GearboxGear::Neutral:
            return 'N';
        case GearboxGear::First:
        case GearboxGear::Second:
        case GearboxGear::Third:
        case GearboxGear::Fourth:
        case GearboxGear::Fifth:
        case GearboxGear::Sixth:
        case GearboxGear::Seventh:
            switch(this->profile_id) {
                case 0:
                    return 'A';
                case 1:
                    return 'B';
                case 2:
                    return 'C';
                default:
                    return 'D';
            }
        case GearboxGear::SignalNotAvaliable:
            return 0xFF;
        default:
            return ' ';
    }
}

bool AgilityProfile::should_upshift(GearboxGear current_gear) {
    return false;
}

bool AgilityProfile::should_downshift(GearboxGear current_gear) {
    return false;
}

char ComfortProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (target) {
        case GearboxGear::Park:
            return 'P';
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return 'R';
        case GearboxGear::Neutral:
            return 'N';
        case GearboxGear::First:
        case GearboxGear::Second:
        case GearboxGear::Third:
        case GearboxGear::Fourth:
        case GearboxGear::Fifth:
        case GearboxGear::Sixth:
        case GearboxGear::Seventh:
            switch(this->profile_id) {
                case 0:
                    return 'A';
                case 1:
                    return 'B';
                case 2:
                    return 'C';
                default:
                    return 'D';
            }
        case GearboxGear::SignalNotAvaliable:
            return 0xFF;
        default:
            return ' ';
    }
}

bool ComfortProfile::should_upshift(GearboxGear current_gear) {
    return false;
}

bool ComfortProfile::should_downshift(GearboxGear current_gear) {
    return false;
}

char WinterProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (target) {
        case GearboxGear::Park:
            return 'P';
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return 'R';
        case GearboxGear::Neutral:
            return 'N';
        case GearboxGear::First:
        case GearboxGear::Second:
        case GearboxGear::Third:
        case GearboxGear::Fourth:
        case GearboxGear::Fifth:
        case GearboxGear::Sixth:
        case GearboxGear::Seventh:
            switch(this->profile_id) {
                case 0:
                    return 'A';
                case 1:
                    return 'B';
                case 2:
                    return 'C';
                default:
                    return 'D';
            }
        case GearboxGear::SignalNotAvaliable:
            return 0xFF;
        default:
            return ' ';
    }
}

bool WinterProfile::should_upshift(GearboxGear current_gear) {
    return false;
}

bool WinterProfile::should_downshift(GearboxGear current_gear) {
    return false;
}

char StandardProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (target) {
        case GearboxGear::Park:
            return 'P';
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return 'R';
        case GearboxGear::Neutral:
            return 'N';
        case GearboxGear::First:
        case GearboxGear::Second:
        case GearboxGear::Third:
        case GearboxGear::Fourth:
        case GearboxGear::Fifth:
        case GearboxGear::Sixth:
        case GearboxGear::Seventh:
            switch(this->profile_id) {
                case 0:
                    return 'A';
                case 1:
                    return 'B';
                case 2:
                    return 'C';
                default:
                    return 'D';
            }
        case GearboxGear::SignalNotAvaliable:
            return 0xFF;
        default:
            return ' ';
    }
}

bool StandardProfile::should_upshift(GearboxGear current_gear) {
    return false;
}

bool StandardProfile::should_downshift(GearboxGear current_gear) {
    return false;
}

char ManualProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (target) {
        case GearboxGear::Park:
            return 'P';
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return 'R';
        case GearboxGear::Neutral:
            return 'N';
        case GearboxGear::First:
            return '1';
        case GearboxGear::Second:
            return '2';
        case GearboxGear::Third:
            return '3';
        case GearboxGear::Fourth:
            return '4';
        case GearboxGear::Fifth:
            return '5';
        case GearboxGear::Sixth:
            return '6';
        case GearboxGear::Seventh:
            return '7';
        case GearboxGear::SignalNotAvaliable:
            return 0xFF;
        default:
            return ' ';
    }
}

bool ManualProfile::should_upshift(GearboxGear current_gear) {
    return false;
}

bool ManualProfile::should_downshift(GearboxGear current_gear) {
    return false;
}
