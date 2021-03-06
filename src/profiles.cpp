#include "profiles.h"
#include <gearbox_config.h>
#include "adv_opts.h"
#include "tcm_maths.h"
#include "gearbox.h"

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
    float dp = ((float)(sensors->pedal_pos*100)/250.0f);
    if (dp > 10) {
        dp = 10;
    }
    if (dp == 0) {
        dp = 1;
    }
    dp *=10.0;
    return ShiftCharacteristics {
        .target_d_rpm = 60,
        .shift_speed = dp,
    };
}

bool AgilityProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return standard->should_upshift(current_gear, sensors);
}

bool AgilityProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    return standard->should_downshift(current_gear, sensors);
}

TccLockupBounds AgilityProfile::get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) {
    return TccLockupBounds {
        .max_slip_rpm = (int)MAX(70, sensors->static_torque),
        .min_slip_rpm = (int)MAX(10, sensors->static_torque*0.75)
    };
}


ShiftCharacteristics ComfortProfile::get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) {
    return ShiftCharacteristics {
        .target_d_rpm = 30,
        .shift_speed = 2.0,
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

TccLockupBounds ComfortProfile::get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) {
    return TccLockupBounds {
        .max_slip_rpm = (int)MAX(100, sensors->static_torque*1.2),
        .min_slip_rpm = (int)MAX(50, sensors->static_torque)
    };
}

ShiftCharacteristics WinterProfile::get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) {
    return ShiftCharacteristics {
        .target_d_rpm = 20,
        .shift_speed = 3.0,
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
#ifdef MANUAL_AUTO_DOWNSHIFT
    if (current_gear == GearboxGear::Second) {
        return false;
    }
    return manual->should_downshift(current_gear, sensors);
#else
    return false;
#endif
}

// Minimum lockup
TccLockupBounds WinterProfile::get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) {
    return TccLockupBounds {
        .max_slip_rpm = (int)MAX(100, sensors->static_torque*1.5),
        .min_slip_rpm = (int)MAX(50, sensors->static_torque)
    };
}



void StandardProfile::on_upshift_complete(ShiftResponse resp, uint8_t from_gear, SensorData* sensors) {
    
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
        .target_d_rpm = 50,
        .shift_speed = 5.0,
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
    if (curr_rpm >= Gearbox::redline_rpm) {
        return true;
    }
    if (sensors->pedal_pos == 0 || sensors->is_braking) { // Don't upshift if not pedal or braking
        return false;
    }
    float pedal_perc = ((float)sensors->pedal_pos*100)/250.0;
    float rpm_percent = (float)(sensors->input_rpm-1000)*100.0 / (float)(Gearbox::redline_rpm-1000);
    int rpm_threshold = 0;
    // Load (idx) vs pedal
    if (current_gear == GearboxGear::First) {
        rpm_threshold = 2200;
    } else if (current_gear == GearboxGear::Second) {
        rpm_threshold = 2100;
    } else if (current_gear == GearboxGear::Third) {
        rpm_threshold = 2000;
    } else if (current_gear == GearboxGear::Fourth) {
        rpm_threshold = 1900;
    }
    unsigned long t =  esp_timer_get_time()/1000;
    if (curr_rpm > rpm_threshold && pedal_perc <= rpm_percent && t-sensors->last_shift_time > 2000) {
        return true;
    }
    return false;
}

bool StandardProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) { return false; }
    float pedal_perc = ((float)sensors->pedal_pos*100)/250.0;
    float rpm_percent = (float)(sensors->input_rpm-MIN_WORKING_RPM)*100.0/(float)(Gearbox::redline_rpm-MIN_WORKING_RPM);
    if (current_gear == GearboxGear::Second && (sensors->input_rpm > 300 || sensors->engine_rpm > 800)) {
        return false;
    }
    if (sensors->input_rpm < MIN_WORKING_RPM && sensors->engine_rpm < MIN_WORKING_RPM) {
        return true;
    }
    else if (sensors->input_rpm < Gearbox::redline_rpm/2 && sensors->engine_rpm < Gearbox::redline_rpm/2 && pedal_perc >= rpm_percent*4) {
        if (current_gear == GearboxGear::Second) {
            return false;
        }
        return true;
    } else {
        return false;
    }
   return false;
}

TccLockupBounds StandardProfile::get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) {
    return TccLockupBounds {
        .max_slip_rpm = (int)MAX(50, sensors->static_torque),
        .min_slip_rpm = (int)MAX(1, sensors->static_torque/2)
    };
}

ShiftCharacteristics ManualProfile::get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) {
    return ShiftCharacteristics {
        .target_d_rpm = 70,
        .shift_speed = 20.0,
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
#ifdef MANUAL_AUTO_DOWNSHIFT
    if (current_gear == GearboxGear::First) {
        return false;
    } else if (sensors->input_rpm < 300 && sensors->engine_rpm < MIN_WORKING_RPM && sensors->pedal_pos == 0) {
        return true;
    }
    return false;
#else
    return false;
#endif
}

TccLockupBounds ManualProfile::get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) {
    return TccLockupBounds {
        .max_slip_rpm = (int)MAX(30, sensors->static_torque/2),
        .min_slip_rpm = (int)MAX(0, sensors->static_torque/4)
    };
}


AgilityProfile* agility = new AgilityProfile();
ComfortProfile* comfort = new ComfortProfile();
WinterProfile* winter = new WinterProfile();
ManualProfile* manual = new ManualProfile();
StandardProfile* standard = new StandardProfile();
