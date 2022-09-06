#include "profiles.h"
#include <gearbox_config.h>
#include "adv_opts.h"
#include "tcm_maths.h"
#include "gearbox.h"
#include "maps.h"


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
    float dp = ((float)(sensors->pedal_pos*10)/250.0f);
    if (dp > 10) {
        dp = 10;
    }
    if (dp == 0) {
        dp = 1;
    }
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
        .min_slip_rpm = (int)MAX(10, sensors->static_torque*0.25)
    };
}

ComfortProfile::ComfortProfile(bool is_diesel) : AbstractProfile() {
    // Actual gear
    //TcmMap(uint16_t X_Size, uint16_t Y_size, int16_t* x_ids, int16_t* y_ids);
    this->upshift_table = new TcmMap(11, 4, shift_table_x_header, upshift_y_headers);
    this->downshift_table = new TcmMap(11, 4, shift_table_x_header, downshift_y_headers);

    if (!this->upshift_table->allocate_ok() || !this->downshift_table->allocate_ok()) {
        ESP_LOGE("COMFORT", "Upshift/Downshift map allocation failed!");
        delete this->upshift_table;
        delete this->downshift_table;
    } else {
        if (is_diesel) { // DIESEL PROFILE
            if (this->upshift_table->add_data(C_DIESEL_UPSHIFT_MAP, 44) && this->downshift_table->add_data(C_DIESEL_DOWNSHIFT_MAP, 44)) {
                ESP_LOGI("COMFORT", "Upshift and downshift maps loaded OK!");
            } else {
                ESP_LOGE("COMFORT", "Upshift/Downshift map data add failed!");
            }
        } else { // PETROL PROFILE
            if (this->upshift_table->add_data(C_DIESEL_UPSHIFT_MAP, 44) && this->downshift_table->add_data(C_PETROL_DOWNSHIFT_MAP, 44)) {
                ESP_LOGI("COMFORT", "Upshift and downshift maps loaded OK!");
            } else {
                ESP_LOGE("COMFORT", "Upshift/Downshift map data add failed!");
            }
        }
    }
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
    if (this->upshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm > this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    }
    return standard->should_upshift(current_gear, sensors);
}

bool ComfortProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Second || current_gear == GearboxGear::First) {
        return false;
    }
    
    if (this->downshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm < this->downshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    }

    return standard->should_downshift(current_gear, sensors);
    
}

TccLockupBounds ComfortProfile::get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) {
    return TccLockupBounds {
        .max_slip_rpm = 50,
        .min_slip_rpm = 10
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
        rpm_threshold = 1500;
    }
    if (curr_rpm > rpm_threshold && pedal_perc <= rpm_percent && sensors->current_timestamp_ms-sensors->last_shift_time > 500) {
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
        .shift_speed = 9.0,
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

/*
AgilityProfile* agility = new AgilityProfile();
ComfortProfile* comfort = new ComfortProfile();
WinterProfile* winter = new WinterProfile();
ManualProfile* manual = new ManualProfile();
StandardProfile* standard = new StandardProfile();
*/

/* Now initialized in main.cpp */
AgilityProfile* agility = nullptr;
ComfortProfile* comfort = nullptr;
WinterProfile* winter = nullptr;
ManualProfile* manual = nullptr;
StandardProfile* standard = nullptr;
