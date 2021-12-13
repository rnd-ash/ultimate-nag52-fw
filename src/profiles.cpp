#include "profiles.h"

// REMEMBER
// Higher number = LESS pressure

// LOAD 0% 10% 20% 30% 40% 50% 60% 70% 80% 90% 100%
// Any load can be calibrated
const uint16_t map_1_2[11] = {290, 285, 280, 275, 270, 265, 260, 255, 250, 240, 230};
const uint16_t map_2_3[11] = {330, 320, 300, 280, 260, 240, 220, 200, 160, 140, 120};
const uint16_t map_3_4[11] = {300, 295, 290, 285, 280, 275, 270, 260, 250, 240, 200};
const uint16_t map_4_5[11] = {250, 225, 220, 200, 180, 160, 140, 120, 100, 80, 60};

const uint16_t map_2_1[11] = {300, 280, 260, 240, 220, 200, 180, 160, 140, 120, 100};
const uint16_t map_3_2[11] = {400, 390, 380, 370, 360, 350, 340, 330, 320, 310, 300}; // BEEFY
const uint16_t map_4_3[11] = {350, 325, 300, 275, 250, 225, 200, 175, 150, 125, 100};
const uint16_t map_5_4[11] = {350, 325, 300, 275, 250, 225, 200, 175, 150, 125, 100};

// 0 RPM, 1000RPM, 2000RPM Calibration, 3000RPM, 4000RPM, 5000RPM, 6000RPM, 7000RPM
// 1800-2200 RPM is where adaptation can occur
const float pressure_rpm_normalizer[8] = { 0.8, 0.9, 1, 1.05, 1.1, 1.15, 1.2, 1.25 };

// -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120
// 0-90 is where adaptation can occur
const float pressure_temp_normalizer[17] = { 
    0.9, 0.9, 0.9, 0.9, 0.9, // -40-0C
    0.95, 0.95, 0.95, 0.95, 0.95, // 0-50C
    1.0, 1.0, 1.0, 1.0, 1.05, 1.05, 1.05 }; //50C+

uint16_t correct_pressure_temp(uint16_t map_pressure, SensorData* sensors) {
    int atf_temp_c = sensors->atf_temp-50;
    if (atf_temp_c < -40) { return pressure_temp_normalizer[0]; }
    else if (atf_temp_c > 120) { return pressure_temp_normalizer[16]; }
    int min = (atf_temp_c/10) + 4;
    int max = min+1;
    float dy = pressure_temp_normalizer[max] - pressure_temp_normalizer[min];
    float dx = (max-min)*10;
    return map_pressure * (pressure_temp_normalizer[min] + ((dy/dx)) * ((atf_temp_c+40)-(min*10)));
}

uint16_t correct_pressure_rpm(uint16_t map_pressure, SensorData* sensors) {
    int m = sensors->engine_rpm; // Check who is driving the line pressure
    if (sensors->input_rpm > sensors->engine_rpm) {
        m = sensors->input_rpm;
    }
    if (m > 7000) {return correct_pressure_temp(pressure_rpm_normalizer[7], sensors); } 
    else if (m < 0) { return correct_pressure_temp(pressure_rpm_normalizer[0], sensors); }
    int min = m/1000;
    int max = min+1;
    float dy = pressure_rpm_normalizer[max] - pressure_rpm_normalizer[min];
    float dx = (max-min)*1000;
    return correct_pressure_temp(map_pressure * (pressure_rpm_normalizer[min] + ((dy/dx)) * (m-(min*1000))), sensors);
}

uint16_t find_map_value(const uint16_t* map, int ped, SensorData* sensors) {
    if (ped <= 0) {
        return correct_pressure_rpm(map[0], sensors);
    } else if (ped >= 100) {
        return correct_pressure_rpm(map[10], sensors);
    } else {
        // Interpolate
        int min = ped/10;
        int max = min+1;
        if (map[min] == map[max]) {
            return correct_pressure_rpm(map[min], sensors); // shortcut
        } else {
            float dy = map[max] - map[min];
            float dx = (max-min)*10;
            return correct_pressure_rpm(map[min] + ((dy/dx)) * (ped-(min*10)), sensors);
        }     
    }
}

char AgilityProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
   switch (actual) {
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

ShiftData AgilityProfile::get_shift_data(ProfileGearChange requested, SensorData* sensors) {
    int pedal_perc = (int)(sensors->pedal_pos*100) / 250;
    if (requested == ProfileGearChange::ONE_TWO) {
        uint16_t r = find_map_value(map_1_2, pedal_perc, sensors);
        return ShiftData { .spc_perc = r, .mpc_perc = r, .targ_ms = 500 };
    } else if (requested == ProfileGearChange::TWO_THREE) {
        uint16_t r = find_map_value(map_2_3, pedal_perc, sensors);
        return ShiftData { .spc_perc = r, .mpc_perc = r, .targ_ms = 500 };
    } else if (requested == ProfileGearChange::THREE_FOUR) {
        uint16_t r = find_map_value(map_3_4, pedal_perc, sensors);
        return ShiftData { .spc_perc = r, .mpc_perc = r, .targ_ms = 500 };
    } else if (requested == ProfileGearChange::FOUR_FIVE) {
        uint16_t r = find_map_value(map_4_5, pedal_perc, sensors);
        return ShiftData { .spc_perc = r, .mpc_perc = r, .targ_ms = 500 };
    } 
    // Downshifts
    else if (requested == ProfileGearChange::TWO_ONE) {
        uint16_t r = find_map_value(map_2_1, pedal_perc, sensors);
        return ShiftData { .spc_perc = r, .mpc_perc = r, .targ_ms = 500 };
    } else if (requested == ProfileGearChange::THREE_TWO) {
        uint16_t r = find_map_value(map_3_2, pedal_perc, sensors);
        return ShiftData { .spc_perc = r, .mpc_perc = r, .targ_ms = 500 };
    } else if (requested == ProfileGearChange::FOUR_THREE) {
        uint16_t r = find_map_value(map_4_3, pedal_perc, sensors);
        return ShiftData { .spc_perc = r, .mpc_perc = r, .targ_ms = 500 };
    } else if (requested == ProfileGearChange::FIVE_FOUR) {
        uint16_t r = find_map_value(map_5_4, pedal_perc, sensors);
        return ShiftData { .spc_perc = r, .mpc_perc = r, .targ_ms = 500 };
    } else {
        return ShiftData { .spc_perc = 100, .mpc_perc = 100, .targ_ms = 0 }; // WTF. Fallback
    }
}

bool AgilityProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool AgilityProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

ShiftData ComfortProfile::get_shift_data(ProfileGearChange requested, SensorData* sensors) {
    ShiftData p = agility->get_shift_data(requested, sensors);
    // Reduce pressure a bit from the test 'agility' dynamic map
    p.mpc_perc += 20;
    p.spc_perc += 20;
    return p;
}

char ComfortProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (actual) {
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

bool ComfortProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool ComfortProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
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

bool WinterProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool WinterProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
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

bool StandardProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool StandardProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
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

bool ManualProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool ManualProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}


AgilityProfile* agility = new AgilityProfile();
ComfortProfile* comfort = new ComfortProfile();
WinterProfile* winter = new WinterProfile();
ManualProfile* manual = new ManualProfile();
StandardProfile* standard = new StandardProfile();
