#include "profiles.h"
#include <gearbox_config.h>
#include "adv_opts.h"
#include "tcu_maths.h"
#include "gearbox.h"
#include "maps.h"

static_assert(SHIFT_MAP_X_SIZE*SHIFT_MAP_Y_SIZE == SHIFT_MAP_SIZE);

AbstractProfile::AbstractProfile(bool is_diesel, 
        const char* tag_id, 
        const char* upshift_map_name_diesel, 
        const char* downshift_map_name_diesel, 
        const char* upshift_map_name_petrol, 
        const char* downshift_map_name_petrol,
        const char* upshift_time_map_name,
        const char* downshift_time_map_name,
        const int16_t* def_upshift_data_diesel,
        const int16_t* def_downshift_data_diesel,
        const int16_t* def_upshift_data_petrol,
        const int16_t* def_downshift_data_petrol,
        const int16_t* def_upshift_time_data,
        const int16_t* def_downshift_time_data
    ) {

    this->is_diesel = is_diesel;
    this->tag_id = tag_id;

    const char* key_name;
    const int16_t* default_map;
    /** Upshift map **/
    if (is_diesel) {
        key_name = upshift_map_name_diesel;
        default_map = def_upshift_data_diesel;
    } else {
        key_name = upshift_map_name_petrol;
        default_map = def_upshift_data_petrol;
    }
    this->upshift_table = new StoredTcuMap(key_name, SHIFT_MAP_SIZE, shift_table_x_header, upshift_y_headers, SHIFT_MAP_X_SIZE, SHIFT_MAP_Y_SIZE, default_map);
    if (!this->upshift_table->init_ok()) {
        delete[] this->upshift_table;
    }

    /** Downshift map **/
    if (is_diesel) {
        key_name = downshift_map_name_diesel;
        default_map = def_downshift_data_diesel;
    } else {
        key_name = downshift_map_name_petrol;
        default_map = def_downshift_data_petrol;
    }
    this->downshift_table = new StoredTcuMap(key_name, SHIFT_MAP_SIZE, shift_table_x_header, upshift_y_headers, SHIFT_MAP_X_SIZE, SHIFT_MAP_Y_SIZE, default_map);
    if (!this->downshift_table->init_ok()) {
        delete[] this->downshift_table;
    }

    // Up/downshift time tables
    int16_t redline = is_diesel ? VEHICLE_CONFIG.red_line_rpm_diesel : VEHICLE_CONFIG.red_line_rpm_petrol;
    int16_t step_size = (redline-1000) / 4;
    int16_t shift_rpm_points[5] = {(int16_t)1000,  (int16_t)(1000+(step_size)), (int16_t)(1000+(step_size*2)), (int16_t)(1000+(step_size*3)), redline};
    this->upshift_time_map = new StoredTcuMap(upshift_time_map_name, SHIFT_TIME_MAP_SIZE, shift_time_table_x_header, (const int16_t*)shift_rpm_points, 6, 5, def_upshift_time_data);
    if (!this->upshift_time_map->init_ok()) {
        delete[] this->upshift_time_map;
    }
    this->downshift_time_map = new StoredTcuMap(downshift_time_map_name, SHIFT_TIME_MAP_SIZE, shift_time_table_x_header, (const int16_t*)shift_rpm_points, 6, 5, def_downshift_time_data);
    if (!this->downshift_time_map->init_ok()) {
        delete[] this->downshift_time_map;
    }
}

void AbstractProfile::reload_data() {
    if (this->upshift_table) {
        this->upshift_table->reload_from_eeprom();
    }
    if (this->downshift_table) {
        this->downshift_table->reload_from_eeprom();
    }
}


AgilityProfile::AgilityProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "WINTER", 
        MAP_NAME_A_DIESEL_UPSHIFT, 
        MAP_NAME_A_DIESEL_DOWNSHIFT,
        MAP_NAME_A_PETROL_UPSHIFT, 
        MAP_NAME_A_PETROL_DOWNSHIFT,
        MAP_NAME_A_UPSHIFT_TIME,
        MAP_NAME_A_DOWNSHIFT_TIME,
        A_DIESEL_UPSHIFT_MAP,
        A_DIESEL_DOWNSHIFT_MAP,
        A_PETROL_UPSHIFT_MAP,
        A_PETROL_DOWNSHIFT_MAP,
        A_UPSHIFT_TIME_MAP,
        A_DOWNSHIFT_TIME_MAP
    ) {
}


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
    if (current_gear == GearboxGear::Fifth) {
        return false;
    }
    if (this->upshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm > this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    }
    return standard->should_upshift(current_gear, sensors);
}

bool AgilityProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) {
        return false;
    }
    if (this->downshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm < this->downshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    }
    return standard->should_downshift(current_gear, sensors);
}

TccLockupBounds AgilityProfile::get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) {
    return TccLockupBounds {
        .max_slip_rpm = (int)MAX(70, sensors->static_torque),
        .min_slip_rpm = (int)MAX(10, sensors->static_torque*0.25)
    };
}

ComfortProfile::ComfortProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "COMFORT", 
        MAP_NAME_C_DIESEL_UPSHIFT, 
        MAP_NAME_C_DIESEL_DOWNSHIFT,
        MAP_NAME_C_PETROL_UPSHIFT, 
        MAP_NAME_C_PETROL_DOWNSHIFT,
        MAP_NAME_C_UPSHIFT_TIME,
        MAP_NAME_C_DOWNSHIFT_TIME,
        C_DIESEL_UPSHIFT_MAP,
        C_DIESEL_DOWNSHIFT_MAP,
        C_PETROL_UPSHIFT_MAP,
        C_PETROL_DOWNSHIFT_MAP,
        C_UPSHIFT_TIME_MAP,
        C_DOWNSHIFT_TIME_MAP
    ) {
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

WinterProfile::WinterProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "WINTER", 
        MAP_NAME_M_DIESEL_UPSHIFT, 
        MAP_NAME_M_DIESEL_DOWNSHIFT,
        MAP_NAME_M_PETROL_UPSHIFT, 
        MAP_NAME_M_PETROL_DOWNSHIFT,
        MAP_NAME_M_UPSHIFT_TIME,
        MAP_NAME_M_DOWNSHIFT_TIME,
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP,
        M_UPSHIFT_TIME_MAP,
        M_DOWNSHIFT_TIME_MAP
    ) {
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
    if (current_gear == GearboxGear::Second) {
        return false;
    }
    return manual->should_downshift(current_gear, sensors);
}

// Minimum lockup
TccLockupBounds WinterProfile::get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) {
    return TccLockupBounds {
        .max_slip_rpm = (int)MAX(100, sensors->static_torque*1.5),
        .min_slip_rpm = (int)MAX(50, sensors->static_torque)
    };
}

StandardProfile::StandardProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "STANDARD", 
        MAP_NAME_S_DIESEL_UPSHIFT, 
        MAP_NAME_S_DIESEL_DOWNSHIFT,
        MAP_NAME_S_PETROL_UPSHIFT, 
        MAP_NAME_S_PETROL_DOWNSHIFT,
        MAP_NAME_S_UPSHIFT_TIME,
        MAP_NAME_S_DOWNSHIFT_TIME,
        S_DIESEL_UPSHIFT_MAP,
        S_DIESEL_DOWNSHIFT_MAP,
        S_PETROL_UPSHIFT_MAP,
        S_PETROL_DOWNSHIFT_MAP,
        S_UPSHIFT_TIME_MAP,
        S_DOWNSHIFT_TIME_MAP
    ) {
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
    if (this->upshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm > this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    }
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
    if (this->downshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm < this->downshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    }
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

ManualProfile::ManualProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "MANUAL", 
        MAP_NAME_M_DIESEL_UPSHIFT, 
        MAP_NAME_M_DIESEL_DOWNSHIFT,
        MAP_NAME_M_PETROL_UPSHIFT, 
        MAP_NAME_M_PETROL_DOWNSHIFT,
        MAP_NAME_W_UPSHIFT_TIME,
        MAP_NAME_W_DOWNSHIFT_TIME,
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP,
        W_UPSHIFT_TIME_MAP,
        W_DOWNSHIFT_TIME_MAP
    ) {
}

bool ManualProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool ManualProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) {
        return false;
    } else if (sensors->input_rpm < 300 && sensors->engine_rpm < MIN_WORKING_RPM && sensors->pedal_pos == 0) {
        return true;
    }
    return false;
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
