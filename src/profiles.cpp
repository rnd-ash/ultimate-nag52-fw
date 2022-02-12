#include "profiles.h"
#include <gearbox_config.h>

GearboxDisplayGear AgilityProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
   switch (target) {
        case GearboxGear::Park:
            return GearboxDisplayGear::P;
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return GearboxDisplayGear::R;
        case GearboxGear::Neutral:
            return GearboxDisplayGear::N;
        case GearboxGear::First:
            return GearboxDisplayGear::One;
        case GearboxGear::Second:
            return GearboxDisplayGear::Two;
        case GearboxGear::Third:
            return GearboxDisplayGear::Three;
        case GearboxGear::Fourth:
            return GearboxDisplayGear::Four;
        case GearboxGear::Fifth:
            return GearboxDisplayGear::Five;
        case GearboxGear::SignalNotAvaliable:
        default:
            return GearboxDisplayGear::SNA;
    }
}


ShiftCharacteristics AgilityProfile::get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) {
    return ShiftCharacteristics {
        .target_shift_time_ms = 500,
        .shift_firmness = 0.9,
        .shift_speed = 0.9,
    };
}

bool AgilityProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return standard->should_upshift(current_gear, sensors);
}

bool AgilityProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    return standard->should_downshift(current_gear, sensors);
}


ShiftCharacteristics ComfortProfile::get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) {
    return ShiftCharacteristics {
        .target_shift_time_ms = 500,
        .shift_firmness = 1.1,
        .shift_speed = 1.1,
    };
}

GearboxDisplayGear ComfortProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (target) {
        case GearboxGear::Park:
            return GearboxDisplayGear::P;
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return GearboxDisplayGear::R;
        case GearboxGear::Neutral:
            return GearboxDisplayGear::N;
        case GearboxGear::First:
            return GearboxDisplayGear::One;
        case GearboxGear::Second:
            return GearboxDisplayGear::Two;
        case GearboxGear::Third:
            return GearboxDisplayGear::Three;
        case GearboxGear::Fourth:
            return GearboxDisplayGear::Four;
        case GearboxGear::Fifth:
            return GearboxDisplayGear::Five;
        case GearboxGear::SignalNotAvaliable:
        default:
            return GearboxDisplayGear::SNA;
    }
}

bool ComfortProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return standard->should_upshift(current_gear, sensors);
}

bool ComfortProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Second || current_gear == GearboxGear::First) {
        return false;
    }
    return standard->should_downshift(current_gear, sensors);
}

ShiftCharacteristics WinterProfile::get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) {
    return ShiftCharacteristics {
        .target_shift_time_ms = 500,
        .shift_firmness = 1.1,
        .shift_speed = 1.1,
    };
}

GearboxDisplayGear WinterProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (target) {
        case GearboxGear::Park:
            return GearboxDisplayGear::P;
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return GearboxDisplayGear::R;
        case GearboxGear::Neutral:
            return GearboxDisplayGear::N;
        case GearboxGear::First:
            return GearboxDisplayGear::One;
        case GearboxGear::Second:
            return GearboxDisplayGear::Two;
        case GearboxGear::Third:
            return GearboxDisplayGear::Three;
        case GearboxGear::Fourth:
            return GearboxDisplayGear::Four;
        case GearboxGear::Fifth:
            return GearboxDisplayGear::Five;
        case GearboxGear::SignalNotAvaliable:
        default:
            return GearboxDisplayGear::SNA;
    }
}

bool WinterProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool WinterProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

void StandardProfile::on_upshift_complete(ShiftResponse resp, uint8_t from_gear, SensorData* sensors) {
    ESP_LOGI("ADAPT", "Adaptation called. Shifted? %d, Measured shift? %d, Shift time %d ms, Shift d_rpm avg: %d", 
        resp.shifted, resp.valid_measurement, resp.time_ms, resp.avg_d_rpm
    );
    /*
    float* target;
    switch(from_gear) {
        case 1:
            target = &adaptation_1_2;
            break;
        case 2:
            target = &adaptation_2_3;
            break;
        case 3:
            target = &adaptation_3_4;
            break;
        case 4:
            target = &adaptation_4_5;
            break;
        default:
            return;
    }
    int min_time = 500;
    int max_time = 600;
    if (from_gear == 1 || from_gear == 4) {
        min_time = 400;
        max_time = 500;
    }
    if (resp.shifted && resp.valid_measurement) {
        if (resp.time_ms < min_time) {
            *target += 0.01;
        } else if (resp.time_ms > max_time) {
            *target -= 0.01;
        }
    }
    ESP_LOGI("ADAPT", "Adaptation block is now %.2f %.2f %.2f %.2f", adaptation_1_2, adaptation_2_3, adaptation_3_4, adaptation_4_5);
    */
}

ShiftCharacteristics StandardProfile::get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) {
    return ShiftCharacteristics {
        .target_shift_time_ms = 500,
        .shift_firmness = 1,
        .shift_speed = 1,
    };
}

GearboxDisplayGear StandardProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (target) {
        case GearboxGear::Park:
            return GearboxDisplayGear::P;
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return GearboxDisplayGear::R;
        case GearboxGear::Neutral:
            return GearboxDisplayGear::N;
        case GearboxGear::First:
            return GearboxDisplayGear::One;
        case GearboxGear::Second:
            return GearboxDisplayGear::Two;
        case GearboxGear::Third:
            return GearboxDisplayGear::Three;
        case GearboxGear::Fourth:
            return GearboxDisplayGear::Four;
        case GearboxGear::Fifth:
            return GearboxDisplayGear::Five;
        case GearboxGear::SignalNotAvaliable:
        default:
            return GearboxDisplayGear::SNA;
    }
}

bool StandardProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Fifth) { return false; }
    int curr_rpm = sensors->input_rpm;
    if (curr_rpm > 4500) { // Protect da engine
        return true;
    }
    if (sensors->pedal_pos == 0 || sensors->is_braking) { // Don't upshift if not pedal or braking
        return false;
    }
    float pedal_perc = ((float)sensors->pedal_pos*100)/250.0;
    float rpm_percent = (float)(sensors->input_rpm-1000)*100.0 / (float)(4500-1000);
    int rpm_threshold = 0;
    // Load (idx) vs pedal
    if (current_gear == GearboxGear::First) {
        rpm_threshold = 1500;
    } else if (current_gear == GearboxGear::Second) {
        rpm_threshold = 1500;
    } else if (current_gear == GearboxGear::Third) {
        rpm_threshold = 1500;
    } else if (current_gear == GearboxGear::Fourth) {
        rpm_threshold = 1500;
    }
    unsigned long t =  esp_timer_get_time()/1000;

    if (curr_rpm > rpm_threshold && pedal_perc <= rpm_percent && t-sensors->last_shift_time > 2000) {
        return true;
    }
    return false;
}

bool StandardProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Second || current_gear == GearboxGear::First) { return false; }
    float pedal_perc = ((float)sensors->pedal_pos*100)/250.0;
    float rpm_percent = (float)(sensors->input_rpm-1000)*100.0/(float)(4500-1000);
    unsigned long t =  esp_timer_get_time()/1000;
    if (sensors->input_rpm < 1000 && current_gear != GearboxGear::Third && t-sensors->last_shift_time > 2000) {
        return true;
    } else if (sensors->output_rpm < 200 && current_gear == GearboxGear::Third && t-sensors->last_shift_time > 2000) { // 3-2 downshift only at low speeds if idle
        return true;
    }
    else if (sensors->input_rpm < 2000 && pedal_perc > 60 && pedal_perc >= rpm_percent*2 && t-sensors->last_shift_time > 2000) {
        if (current_gear == GearboxGear::Second) { // Prevent 2-1 downshift (Too twitchy)
            return false;
        }
        return true;
    } else {
        return false;
    }
   return false;
}

ShiftCharacteristics ManualProfile::get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) {
    return ShiftCharacteristics {
        .target_shift_time_ms = 500,
        .shift_firmness = 0.9,
        .shift_speed = 0.9,
    };
}

GearboxDisplayGear ManualProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    switch (target) {
        case GearboxGear::Park:
            return GearboxDisplayGear::P;
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            return GearboxDisplayGear::R;
        case GearboxGear::Neutral:
            return GearboxDisplayGear::N;
        case GearboxGear::First:
            return GearboxDisplayGear::One;
        case GearboxGear::Second:
            return GearboxDisplayGear::Two;
        case GearboxGear::Third:
            return GearboxDisplayGear::Three;
        case GearboxGear::Fourth:
            return GearboxDisplayGear::Four;
        case GearboxGear::Fifth:
            return GearboxDisplayGear::Five;
        case GearboxGear::SignalNotAvaliable:
        default:
            return GearboxDisplayGear::SNA;
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
