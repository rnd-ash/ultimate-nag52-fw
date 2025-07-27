#include "profiles.h"
#include "adv_opts.h"
#include "tcu_maths.h"
#include "gearbox.h"
#include "maps.h"
#include "nvs/all_keys.h"
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
    this->upshift_table = new StoredMap(key_name, SHIFT_MAP_SIZE, shift_table_x_header, upshift_y_headers, SHIFT_MAP_X_SIZE, SHIFT_MAP_Y_SIZE, default_map);
    if (this->upshift_table->init_status() != ESP_OK) {
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
    this->downshift_table = new StoredMap(key_name, SHIFT_MAP_SIZE, shift_table_x_header, downshift_y_headers, SHIFT_MAP_X_SIZE, SHIFT_MAP_Y_SIZE, default_map);
    if (this->downshift_table->init_status() != ESP_OK) {
        delete[] this->downshift_table;
    }

    // Up/downshift time tables
    int16_t redline = is_diesel ? VEHICLE_CONFIG.red_line_rpm_diesel : VEHICLE_CONFIG.red_line_rpm_petrol;
    int16_t step_size = (redline-1000) / 4;
    int16_t shift_rpm_points[5] = {(int16_t)1000,  (int16_t)(1000+(step_size)), (int16_t)(1000+(step_size*2)), (int16_t)(1000+(step_size*3)), redline};
    this->upshift_time_map = new StoredMap(upshift_time_map_name, SHIFT_TIME_MAP_SIZE, shift_time_table_x_header, const_cast<int16_t*>(shift_rpm_points), 6, 5, def_upshift_time_data);
    if (this->upshift_time_map->init_status() != ESP_OK) {
        delete[] this->upshift_time_map;
    }
    this->downshift_time_map = new StoredMap(downshift_time_map_name, SHIFT_TIME_MAP_SIZE, shift_time_table_x_header, const_cast<int16_t*>(shift_rpm_points), 6, 5, def_downshift_time_data);
    if (this->downshift_time_map->init_status() != ESP_OK) {
        delete[] this->downshift_time_map;
    }
}

ShiftCharacteristics AbstractProfile::get_shift_characteristics(GearChange requested, SensorData* sensors) {
    ShiftCharacteristics result;
    switch (requested) {
        case GearChange::_1_2:
        case GearChange::_2_3:
        case GearChange::_3_4:
        case GearChange::_4_5:
            result.target_shift_time = this->get_upshift_time(sensors->input_rpm, ((float)sensors->pedal_pos*100.0)/250.0);
            break;
        case GearChange::_5_4:
        case GearChange::_4_3:
        case GearChange::_3_2:
        case GearChange::_2_1:
            result.target_shift_time = this->get_downshift_time(sensors->input_rpm, ((float)sensors->pedal_pos*100.0)/250.0);
            break;
        default:
            result.target_shift_time = 500;
            break;
    }
    return result;
}


AgilityProfile::AgilityProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "WINTER", 
        NVS_KEY_MAP_NAME_A_DIESEL_UPSHIFT, 
        NVS_KEY_MAP_NAME_A_DIESEL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_A_PETROL_UPSHIFT, 
        NVS_KEY_MAP_NAME_A_PETROL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_A_UPSHIFT_TIME,
        NVS_KEY_MAP_NAME_A_DOWNSHIFT_TIME,
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
        case GearboxGear::SignalNotAvailable:
        default:
            return GearboxDisplayGear::SNA;
    }
}

bool AgilityProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Fifth) {
        return false;
    }
    if (this->upshift_table != nullptr) { // TEST TABLE
        return (int)sensors->input_rpm > this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    } else {
        return false;
    }
}

bool AgilityProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) {
        return false;
    }
    if (this->downshift_table != nullptr) { // TEST TABLE
        return (int)sensors->input_rpm < this->downshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    } else {
        return false;
    }
}

ComfortProfile::ComfortProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "COMFORT", 
        NVS_KEY_MAP_NAME_C_DIESEL_UPSHIFT, 
        NVS_KEY_MAP_NAME_C_DIESEL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_C_PETROL_UPSHIFT, 
        NVS_KEY_MAP_NAME_C_PETROL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_C_UPSHIFT_TIME,
        NVS_KEY_MAP_NAME_C_DOWNSHIFT_TIME,
        C_DIESEL_UPSHIFT_MAP,
        C_DIESEL_DOWNSHIFT_MAP,
        C_PETROL_UPSHIFT_MAP,
        C_PETROL_DOWNSHIFT_MAP,
        C_UPSHIFT_TIME_MAP,
        C_DOWNSHIFT_TIME_MAP
    ) {
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
        case GearboxGear::SignalNotAvailable:
        default:
            return GearboxDisplayGear::SNA;
    }
}

bool ComfortProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Fifth) {
        return false;
    }
    if (this->upshift_table != nullptr) { // TEST TABLE
        bool can_upshift = sensors->input_rpm > this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
        if (sensors->is_braking) { can_upshift = false; }
        //if (can_upshift) {
        //    if (sensors->max_torque != 0) {
        //        float demanded_load = (MAX(sensors->driver_requested_torque, 0) * 100) / sensors->max_torque;
        //        if (demanded_load > 30) {
        //            can_upshift = false;
        //        }
        //    }
        //}
        return can_upshift;
    } else {
        return false;
    }
}

bool ComfortProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) {
        return false;
    }
    if (this->downshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm < this->downshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    } else {
        return false;
    }
}

WinterProfile::WinterProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "WINTER", 
        NVS_KEY_MAP_NAME_M_DIESEL_UPSHIFT, 
        NVS_KEY_MAP_NAME_M_DIESEL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_M_PETROL_UPSHIFT, 
        NVS_KEY_MAP_NAME_M_PETROL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_C_UPSHIFT_TIME,
        NVS_KEY_MAP_NAME_C_DOWNSHIFT_TIME,
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP,
        C_UPSHIFT_TIME_MAP,
        C_DOWNSHIFT_TIME_MAP
    ) {
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
        case GearboxGear::SignalNotAvailable:
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


StandardProfile::StandardProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "STANDARD", 
        NVS_KEY_MAP_NAME_S_DIESEL_UPSHIFT, 
        NVS_KEY_MAP_NAME_S_DIESEL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_S_PETROL_UPSHIFT, 
        NVS_KEY_MAP_NAME_S_PETROL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_S_UPSHIFT_TIME,
        NVS_KEY_MAP_NAME_S_DOWNSHIFT_TIME,
        S_DIESEL_UPSHIFT_MAP,
        S_DIESEL_DOWNSHIFT_MAP,
        S_PETROL_UPSHIFT_MAP,
        S_PETROL_DOWNSHIFT_MAP,
        S_UPSHIFT_TIME_MAP,
        S_DOWNSHIFT_TIME_MAP
    ) {
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
        case GearboxGear::SignalNotAvailable:
        default:
            return GearboxDisplayGear::SNA;
    }
}

bool StandardProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    this->update(sensors);
    if (current_gear == GearboxGear::Fifth) { return false; }
    if (this->upshift_table != nullptr) { // TEST TABLE
        bool can_upshift = sensors->input_rpm > this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
        if (sensors->is_braking) { can_upshift = false; } // Disable when breaking
        //if (this->accel_delta_factor > 100 && sensors->pedal_pos > 64) {
        //    can_upshift = false;
        //}
        // Load check
        if (can_upshift) {
            if (sensors->pedal_delta->get_average() > 20.0) {
                can_upshift = false;
            }
            //if (sensors->max_torque != 0) {
            //    float demanded_load = (MAX(sensors->converted_driver_torque, 0) * 100) / sensors->max_torque;
            //    if (demanded_load > 30) {
            //        can_upshift = false;
            //    }
            //}
        }
        if (can_upshift) {
            // Stop 'sporatic' upshifting when the user lets go of the pedal quickly (This will trigger a brief wait period)
            if (sensors->pedal_delta->get_average() < -10.0) {
                can_upshift = false;
            }
        }
        return can_upshift;
    } else {
        return false;
    }
}

void StandardProfile::update(SensorData* sensors) {
    // Every 250ms we check sensor inputs
    if (GET_CLOCK_TIME() - this->last_check > 250) {
        this->last_check = GET_CLOCK_TIME();
        if (sensors->pedal_pos - last_sensors.pedal_pos > 64) { // More than a 25% jump in pedal in 250ms
            accel_delta_factor += (sensors->pedal_pos - last_sensors.pedal_pos) * 10;
        }
        last_sensors = *sensors;
    }
    if (this->accel_delta_factor > 0) {
        this->accel_delta_factor-=5;
    }
}

bool StandardProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) { return false; }
    if (this->upshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm < this->downshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    } else {
        return false;
    }
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
        case GearboxGear::SignalNotAvailable:
        default:
            return GearboxDisplayGear::SNA;
    }
}

ManualProfile::ManualProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        "MANUAL", 
        NVS_KEY_MAP_NAME_M_DIESEL_UPSHIFT, 
        NVS_KEY_MAP_NAME_M_DIESEL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_M_PETROL_UPSHIFT, 
        NVS_KEY_MAP_NAME_M_PETROL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_M_UPSHIFT_TIME,
        NVS_KEY_MAP_NAME_M_DOWNSHIFT_TIME,
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP,
        M_UPSHIFT_TIME_MAP,
        M_DOWNSHIFT_TIME_MAP
    ) {
}

bool ManualProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool ManualProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) {
        return false;
    } else if (sensors->input_rpm < 300 && sensors->pedal_pos == 0) {
        return true;
    } else {
        return false;
    }
}


RaceProfile::RaceProfile(bool is_diesel): AbstractProfile(
        is_diesel,
        "RACE", 
        NVS_KEY_MAP_NAME_M_DIESEL_UPSHIFT, 
        NVS_KEY_MAP_NAME_M_DIESEL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_M_PETROL_UPSHIFT, 
        NVS_KEY_MAP_NAME_M_PETROL_DOWNSHIFT,
        NVS_KEY_MAP_NAME_R_UPSHIFT_TIME,
        NVS_KEY_MAP_NAME_R_DOWNSHIFT_TIME,
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP,
        R_UPSHIFT_TIME_MAP,
        R_DOWNSHIFT_TIME_MAP
    ) {
}

GearboxDisplayGear RaceProfile::get_display_gear(GearboxGear target, GearboxGear actual) {
    return manual->get_display_gear(target, actual);
}

bool RaceProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return manual->should_upshift(current_gear, sensors);
}

bool RaceProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    return manual->should_downshift(current_gear, sensors);
}

/* Now initialized in main.cpp */
AgilityProfile* agility = nullptr;
ComfortProfile* comfort = nullptr;
WinterProfile* winter = nullptr;
ManualProfile* manual = nullptr;
StandardProfile* standard = nullptr;
RaceProfile* race = nullptr;

AbstractProfile *profiles[NUM_PROFILES] = {};

void Profiles::init_profiles(bool is_diesel) {
    standard = new StandardProfile(is_diesel);
    comfort = new ComfortProfile(is_diesel);
    winter = new WinterProfile(is_diesel);
    agility = new AgilityProfile(is_diesel);
    manual = new ManualProfile(is_diesel);
    race = new RaceProfile(is_diesel);

    profiles[(uint8_t)GearboxProfile::Standard] = standard;
    profiles[(uint8_t)GearboxProfile::Comfort] = comfort;
    profiles[(uint8_t)GearboxProfile::Winter] = winter;
    profiles[(uint8_t)GearboxProfile::Agility] = agility;
    profiles[(uint8_t)GearboxProfile::Manual] = manual;
    profiles[(uint8_t)GearboxProfile::Race] = race;
}