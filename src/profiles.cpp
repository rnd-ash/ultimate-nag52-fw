#include "profiles.h"
#include <gearbox_config.h>
#include "adv_opts.h"
#include "tcm_maths.h"
#include "gearbox.h"
#include "maps.h"

static_assert(SHIFT_MAP_X_SIZE*SHIFT_MAP_Y_SIZE == SHIFT_MAP_SIZE);

AbstractProfile::AbstractProfile(bool is_diesel, 
        const char* tag_id, 
        const char* upshift_map_name_diesel, 
        const char* downshift_map_name_diesel, 
        const char* upshift_map_name_petrol, 
        const char* downshift_map_name_petrol,
        const int16_t* def_upshift_data_diesel,
        const int16_t* def_downshift_data_diesel,
        const int16_t* def_upshift_data_petrol,
        const int16_t* def_downshift_data_petrol
    ) {

    this->is_diesel = is_diesel;
    this->tag_id = tag_id;
    this->upshift_map_name_diesel = upshift_map_name_diesel; 
    this->downshift_map_name_diesel = downshift_map_name_diesel;
    this->upshift_map_name_petrol = upshift_map_name_petrol;
    this->downshift_map_name_petrol = downshift_map_name_petrol;
    this->def_upshift_data_diesel = def_upshift_data_diesel;
    this->def_downshift_data_diesel = def_downshift_data_diesel;
    this->def_upshift_data_petrol = def_upshift_data_petrol;
    this->def_downshift_data_petrol = def_downshift_data_petrol;


    this->upshift_table = new TcmMap(SHIFT_MAP_X_SIZE, SHIFT_MAP_Y_SIZE, shift_table_x_header, upshift_y_headers);
    this->downshift_table = new TcmMap(SHIFT_MAP_X_SIZE, SHIFT_MAP_Y_SIZE, shift_table_x_header, downshift_y_headers);
    int16_t up_map[SHIFT_MAP_SIZE];
    int16_t down_map[SHIFT_MAP_SIZE];
    if (!this->upshift_table->allocate_ok() || !this->downshift_table->allocate_ok()) {
        ESP_LOGE(tag_id, "Upshift/Downshift map allocation failed!");
        delete this->upshift_table;
        delete this->downshift_table;
    } else {
        if (is_diesel) {
            EEPROM::read_nvs_map_data(upshift_map_name_diesel, up_map, def_upshift_data_diesel, SHIFT_MAP_SIZE);
            EEPROM::read_nvs_map_data(downshift_map_name_diesel, down_map, def_downshift_data_diesel, SHIFT_MAP_SIZE);
        } else {
            EEPROM::read_nvs_map_data(upshift_map_name_petrol, up_map, def_upshift_data_petrol, SHIFT_MAP_SIZE);
            EEPROM::read_nvs_map_data(downshift_map_name_petrol, down_map, def_downshift_data_petrol, SHIFT_MAP_SIZE);
        }
        if (this->upshift_table->add_data(up_map, SHIFT_MAP_SIZE) && this->downshift_table->add_data(down_map, SHIFT_MAP_SIZE)) {
            ESP_LOGI(tag_id, "Upshift and downshift maps loaded OK!");
        } else {
            ESP_LOGE(tag_id, "Upshift/Downshift map data add failed!");
            delete this->upshift_table;
            delete this->downshift_table;
        }
    }
}

void AbstractProfile::reload_data() {
    if (this->upshift_table == nullptr || this->downshift_table == nullptr) {
        return;
    }
    int16_t up_map[SHIFT_MAP_SIZE];
    int16_t down_map[SHIFT_MAP_SIZE];
    if (is_diesel) {
        EEPROM::read_nvs_map_data(upshift_map_name_diesel, up_map, def_upshift_data_diesel, SHIFT_MAP_SIZE);
        EEPROM::read_nvs_map_data(downshift_map_name_diesel, down_map, def_downshift_data_diesel, SHIFT_MAP_SIZE);
    } else {
        EEPROM::read_nvs_map_data(upshift_map_name_petrol, up_map, def_upshift_data_petrol, SHIFT_MAP_SIZE);
        EEPROM::read_nvs_map_data(downshift_map_name_petrol, down_map, def_downshift_data_petrol, SHIFT_MAP_SIZE);
    }
    if (this->upshift_table->add_data(up_map, SHIFT_MAP_SIZE) && this->downshift_table->add_data(down_map, SHIFT_MAP_SIZE)) {
        ESP_LOGI(tag_id, "Upshift and downshift maps re-loaded OK!");
    }
}


AgilityProfile::AgilityProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "WINTER", 
        MAP_NAME_A_DIESEL_UPSHIFT, 
        MAP_NAME_A_DIESEL_DOWNSHIFT,
        MAP_NAME_A_PETROL_UPSHIFT, 
        MAP_NAME_A_PETROL_DOWNSHIFT,
        A_DIESEL_UPSHIFT_MAP,
        A_DIESEL_DOWNSHIFT_MAP,
        A_PETROL_UPSHIFT_MAP,
        A_PETROL_DOWNSHIFT_MAP
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
        C_DIESEL_UPSHIFT_MAP,
        C_DIESEL_DOWNSHIFT_MAP,
        C_PETROL_UPSHIFT_MAP,
        C_PETROL_DOWNSHIFT_MAP
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
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP
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

StandardProfile::StandardProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "STANDARD", 
        MAP_NAME_S_DIESEL_UPSHIFT, 
        MAP_NAME_S_DIESEL_DOWNSHIFT,
        MAP_NAME_S_PETROL_UPSHIFT, 
        MAP_NAME_S_PETROL_DOWNSHIFT,
        S_DIESEL_UPSHIFT_MAP,
        S_DIESEL_DOWNSHIFT_MAP,
        S_PETROL_UPSHIFT_MAP,
        S_PETROL_DOWNSHIFT_MAP
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
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP
    ) {
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
