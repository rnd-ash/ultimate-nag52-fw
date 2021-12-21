#include "profiles.h"
#include <gearbox_config.h>

// REMEMBER
// Higher number = LESS pressure

// LOAD 0% 10% 20% 30% 40% 50% 60% 70% 80% 90% 100%
// Any load can be calibrated
pressure_map map_1_2 = {505, 500, 490, 480, 470, 450, 430, 420, 400, 360, 320};
pressure_map map_2_3 = {460, 450, 440, 430, 420, 410, 390, 370, 350, 325, 300};
pressure_map map_3_4 = {435, 440, 430, 420, 410, 390, 370, 350, 330, 310, 290};
pressure_map map_4_5 = {445, 420, 415, 410, 395, 385, 360, 350, 290, 235, 220};

pressure_map map_2_1 = {525, 525, 500, 480, 465, 455, 445, 420, 340, 375, 255};
pressure_map map_3_2 = {385, 380, 375, 360, 350, 340, 325, 315, 305, 290, 290}; // BEEFY
pressure_map map_4_3 = {430, 430, 420, 410, 395, 385, 375, 360, 350, 340, 325};
pressure_map map_5_4 = {408, 410, 395, 385, 385, 370, 360, 360, 350, 340, 325};

// 0 RPM, 1000RPM, 2000RPM Calibration, 3000RPM, 4000RPM, 5000RPM, 6000RPM, 7000RPM
// 2000-4000RPM
const float RPM_CORRECTION[8] = { 0.94, 0.96, 1, 1.02, 1.04, 1.06, 1.08, 1.1 };

// CORRECTION maps use pedal value to see how 'sporty' the shifts should feel
pressure_map AGILITY_CORRECTION = {  +0,  -5, -10, -15, -20, -30, -40, -50, -60, -70, -80};
pressure_map COMFORT_CORRECTION = {  +10, +10, +15, +20, +25, +30, +32, +34, +36, +38, +40};

// -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 10    0, 110, 120
// 0-90 is where adaptation can occur
const float pressure_temp_normalizer[17] = {
    0.8, 0.8, 0.8, 0.8, 0.8, // -40-0C (0-40)
    0.86, 0.88, 0.9, 0.91, 0.93, // 10-50C (50-90)
    0.95, 0.98, 0.1, 1.1, 1.15, 1.2, 1.25 }; //60C+ (100-160)

uint16_t correct_pressure_temp(uint16_t map_pressure, SensorData* sensors) {
    if (sensors->atf_temp < 0) { return pressure_temp_normalizer[0]; }
    else if (sensors->atf_temp > 160) { return pressure_temp_normalizer[16]; }
    int min = sensors->atf_temp/10;
    int max = min+1;
    float dy = pressure_temp_normalizer[max] - pressure_temp_normalizer[min];
    float dx = (max-min)*10;
    return map_pressure * (pressure_temp_normalizer[min] + ((dy/dx)) * (sensors->atf_temp-(min*10)));
}

uint16_t correct_pressure_rpm(uint16_t map_pressure, SensorData* sensors) {
    if (sensors->input_rpm < 0) { return pressure_temp_normalizer[0]; }
    else if (sensors->input_rpm > 7000) { return pressure_temp_normalizer[7]; }
    int min = sensors->input_rpm/1000;
    int max = min+1;
    float dy = RPM_CORRECTION[max] - RPM_CORRECTION[min];
    float dx = (max-min)*1000;
    return map_pressure * (RPM_CORRECTION[min] + ((dy/dx)) * (sensors->input_rpm-(min*1000)));
}

#define GET_MAP(idx, map_ptr) \
    pressure_raw = map[idx]; \
    if (map_ptr != nullptr) { \
        pressure_raw += map_ptr[pedal_correction_idx]; \
    } \

uint16_t find_map_value_spc(const pressure_map map, const pressure_map correction_map, int load_percentage, SensorData* sensors) {
    // Correction is based on pedal position, not torque (More pedal = snappier shift)
    int pedal_percent = (sensors->pedal_pos*100)/250;
    int pedal_correction_idx = pedal_percent/10;
    if (pedal_correction_idx < 0) {
        pedal_correction_idx = 0;
    } else if (pedal_correction_idx >= 10) {
        pedal_correction_idx = 9;
    }
    uint16_t pressure_raw = 0;
    if (load_percentage <= 0) {
        GET_MAP(0, correction_map)
    } else if (load_percentage >= 100) {
        GET_MAP(10, correction_map)
    } else {
        // Interpolate
        int min = load_percentage/10;
        int max = min+1;
        if (map[min] == map[max]) {
            GET_MAP(min, correction_map)
        } else {
            float dy = map[max] - map[min];
            float dx = (max-min)*10;
            pressure_raw = map[min] + (((dy/dx)) * (load_percentage-(min*10)));
            if (correction_map != nullptr) {
                int min_ped = pedal_correction_idx;
                int max_ped = min_ped+1;
                float dy_correction = correction_map[max_ped] - correction_map[min_ped];
                pressure_raw += correction_map[min_ped] + (((dy_correction/dx)) * (pedal_percent-(min_ped*10)));
            }
        }     
    }
    if (pressure_raw < 50) {
        pressure_raw = 50; // Min we can safely do
    }
    return correct_pressure_temp((uint16_t)pressure_raw, sensors);
}

uint16_t find_map_value_mpc(const pressure_map map, int load_percentage, SensorData* sensors) {
    uint16_t pressure_raw = 0;
    if (load_percentage <= 0) {
        pressure_raw = map[0];
    } else if (load_percentage >= 100) {
        pressure_raw = map[10];
    } else {
        // Interpolate
        int min = load_percentage/10;
        int max = min+1;
        if (map[min] == map[max]) {
            pressure_raw = map[min];
        } else {
            float dy = map[max] - map[min];
            float dx = (max-min)*10;
            pressure_raw = map[min] + (((dy/dx)) * (load_percentage-(min*10)));
        }     
    }
    if (pressure_raw < 50) {
        pressure_raw = 50; // Min we can safely do
    }
    return correct_pressure_temp(correct_pressure_rpm(pressure_raw, sensors), sensors);
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


ShiftData AgilityProfile::get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction) {
    return standard->get_shift_data(requested, sensors, AGILITY_CORRECTION);
}

bool AgilityProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool AgilityProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}


ShiftData ComfortProfile::get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction) {
    return standard->get_shift_data(requested, sensors, COMFORT_CORRECTION);
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

ShiftData WinterProfile::get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction_map) {
    return comfort->get_shift_data(requested, sensors, COMFORT_CORRECTION); // Same shift quality as C
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
ShiftData StandardProfile::get_shift_data(ProfileGearChange requested, SensorData* sensors, pressure_map correction_map) {
    int input_shaft_torque = sensors->static_torque; // Engine is making this much torque
    /*
    if (sensors->input_rpm > 100 && sensors->engine_rpm > 100) {
        // engine vs input shaft rotational difference ~= multiplication of torque factor
        //
        // Yes, this isn't accurate if they have been at differential speeds for a while, but it helps
        input_shaft_torque *= (sensors->engine_rpm / sensors->input_rpm);
    }
    */
    int input_shaft_load_perc = (input_shaft_torque*100) / MAX_TORQUE_RATING_NM;
    if (input_shaft_load_perc < 0) {
        input_shaft_load_perc *= -1;
    }
    int pedal_load = (int)(sensors->pedal_pos*100)/250;
    int load = (input_shaft_load_perc+pedal_load)/2;
    if (requested == ProfileGearChange::ONE_TWO) {
        return ShiftData { 
            .spc_perc = find_map_value_spc(map_1_2, correction_map, load, sensors), 
            .mpc_perc = find_map_value_mpc(map_1_2, load, sensors), 
            .targ_ms = 500 
        };
    } else if (requested == ProfileGearChange::TWO_THREE) {
        return ShiftData { 
            .spc_perc = find_map_value_spc(map_2_3, correction_map, load, sensors), 
            .mpc_perc = find_map_value_mpc(map_2_3, load, sensors), 
            .targ_ms = 500 
        };
    } else if (requested == ProfileGearChange::THREE_FOUR) {
        return ShiftData { 
            .spc_perc = find_map_value_spc(map_3_4, correction_map, load, sensors), 
            .mpc_perc = find_map_value_mpc(map_3_4, load, sensors), 
            .targ_ms = 500 
        };
    } else if (requested == ProfileGearChange::FOUR_FIVE) {
       return ShiftData { 
            .spc_perc = find_map_value_spc(map_4_5, correction_map, load, sensors), 
            .mpc_perc = find_map_value_mpc(map_4_5, load, sensors), 
            .targ_ms = 500 
        };
    } 
    // Downshifts
    else if (requested == ProfileGearChange::TWO_ONE) {
        return ShiftData { 
            .spc_perc = find_map_value_spc(map_2_1, correction_map, load, sensors), 
            .mpc_perc = find_map_value_mpc(map_2_1, load, sensors), 
            .targ_ms = 500 
        };
    } else if (requested == ProfileGearChange::THREE_TWO) {
        return ShiftData { 
            .spc_perc = find_map_value_spc(map_3_2, correction_map, load, sensors), 
            .mpc_perc = find_map_value_mpc(map_3_2, load, sensors), 
            .targ_ms = 500 
        };
    } else if (requested == ProfileGearChange::FOUR_THREE) {
        return ShiftData { 
            .spc_perc = find_map_value_spc(map_4_3, correction_map, load, sensors), 
            .mpc_perc = find_map_value_mpc(map_4_3, load, sensors), 
            .targ_ms = 500 
        };
    } else if (requested == ProfileGearChange::FIVE_FOUR) {
        return ShiftData { 
            .spc_perc = find_map_value_spc(map_5_4, correction_map, load, sensors), 
            .mpc_perc = find_map_value_mpc(map_5_4, load, sensors), 
            .targ_ms = 500 
        };
    } else {
        return ShiftData { .spc_perc = 100, .mpc_perc = 100, .targ_ms = 0 }; // WTF. Fallback this is wrong!
    }
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
    if (current_gear == GearboxGear::Fifth) { return false; }
    int curr_rpm = sensors->input_rpm;
    if (curr_rpm > 4000) { // Protect da engine
        last_shift_time = esp_timer_get_time()/1000;
        return true;
    }
    if (sensors->tcc_slip_rpm > 200) {
        return false;
    }
    int load_perc = (sensors->static_torque*100)/MAX_TORQUE_RATING_NM;
    int pedal_perc = ((int)sensors->pedal_pos*100)/250;
    int rpm_threshold = 0;
    int pedal_perc_threshold = 0;
    int load_threshold = 0;
    // Load (idx) vs pedal
    if (current_gear == GearboxGear::First) {
        rpm_threshold = 2000;
    } else if (current_gear == GearboxGear::Second) {
        rpm_threshold = 1900;
    } else if (current_gear == GearboxGear::Third) {
        rpm_threshold = 1800;
    } else if (current_gear == GearboxGear::Fourth) {
        rpm_threshold = 1500;
    }
    unsigned long t =  esp_timer_get_time()/1000;
    if (curr_rpm > rpm_threshold && load_perc <= 30 && pedal_perc <= 40 && last_shift_time-t > 2000) {
        last_shift_time = esp_timer_get_time()/1000;
        return true;
    }
    return false;
}

bool StandardProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) { return false; }
    if (sensors->input_rpm < 1000) {
        return true;
    } else {
        return false;
    }
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
