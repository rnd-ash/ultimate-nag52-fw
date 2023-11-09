// Profile code

#ifndef PROFILES_H
#define PROFILES_H

#include "canbus/can_hal.h"
#include "common_structs.h"
#include "stored_map.h"

#define PROFILE_ID_STANDARD 0
#define PROFILE_ID_COMFORT 1
#define PROFILE_ID_WINTER 2
#define PROFILE_ID_AGILITY 3
#define PROFILE_ID_MANUAL 4
#define PROFILE_ID_RACE 5

#define SHIFT_MAP_X_SIZE 11
#define SHIFT_MAP_Y_SIZE 4
const int16_t shift_table_x_header[SHIFT_MAP_X_SIZE] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
const int16_t upshift_y_headers[SHIFT_MAP_Y_SIZE] = {1,2,3,4};
const int16_t downshift_y_headers[SHIFT_MAP_Y_SIZE] = {2,3,4,5};

const int16_t shift_time_table_x_header[6] = {0, 20, 40, 60, 80, 100};

/**
 * A profile is designed to read the current conditions of the gearbox and request the gearbox to do something
 */
class AbstractProfile {
public:
    static const uint8_t MAX_PROFILES = 4u;

    AbstractProfile(
        bool is_diesel, 
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
    );
    virtual GearboxProfile get_profile(void) const = 0;
    virtual GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) = 0;
    virtual bool should_upshift(GearboxGear current_gear, SensorData* sensors) = 0;
    virtual bool should_downshift(GearboxGear current_gear, SensorData* sensors) = 0;
    ShiftCharacteristics get_shift_characteristics(ProfileGearChange requested, SensorData* sensors);

    StoredMap* get_upshift_map(void) {
        return this->upshift_table;
    }
    StoredMap* get_downshift_map(void) {
        return this->downshift_table;
    }
    StoredMap* get_upshift_time_map(void) {
        return this->upshift_time_map;
    }
    StoredMap* get_downshift_time_map(void) {
        return this->downshift_time_map;
    }

    uint16_t get_upshift_time(uint16_t input_rpm, float pedal_percent) {
        uint16_t result = 750u;
        if (nullptr != this->upshift_time_map) {
            result = (uint16_t)(this->upshift_time_map->get_value(pedal_percent, (float)input_rpm));
        }
        return result;
    }

    uint16_t get_downshift_time(uint16_t input_rpm, float pedal_percent) {
        uint16_t result = 750u;
        if (nullptr != this->downshift_time_map) {
            result = (uint16_t)(this->downshift_time_map->get_value(pedal_percent, (float)input_rpm));
        }
        return result;
    }

    virtual GearboxGear get_start_gear(void) const {
        return GearboxGear::First;
    }
    void increment_subprofile(void) {
        profile_id += 1u;
        if (profile_id >= MAX_PROFILES) {
            profile_id = 0;
        }
    }
    virtual uint8_t get_profile_id(void) = 0;
protected:
    uint8_t profile_id = 0;
    StoredMap* upshift_table = nullptr;
    StoredMap* downshift_table = nullptr;
    StoredMap* upshift_time_map = nullptr;
    StoredMap* downshift_time_map = nullptr;
private:
    bool is_diesel; 
    const char* tag_id;
};

class AgilityProfile : public AbstractProfile {
public:
    explicit AgilityProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Agility; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    uint8_t get_profile_id() override { return PROFILE_ID_AGILITY; }
};

class ComfortProfile : public AbstractProfile {
public:
    explicit ComfortProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Comfort; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    uint8_t get_profile_id() override { return PROFILE_ID_COMFORT; }
    GearboxGear get_start_gear() const override {
        return GearboxGear::Second;
    }
};

class WinterProfile : public AbstractProfile {
public:
    explicit WinterProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Winter; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    uint8_t get_profile_id() override { return PROFILE_ID_WINTER; }
    GearboxGear get_start_gear() const override {
        return GearboxGear::Second;
    }
};

class StandardProfile : public AbstractProfile {
public:
    explicit StandardProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Standard; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    uint8_t get_profile_id() override { return PROFILE_ID_STANDARD; }
private:
    void update(SensorData *sd);
    int32_t accel_delta_factor = 0;
    SensorData last_sensors = {};
    uint32_t last_check = 0;
};

class ManualProfile : public AbstractProfile {
public:
    explicit ManualProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Manual; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    uint8_t get_profile_id() override { return PROFILE_ID_MANUAL; }
};

class RaceProfile : public AbstractProfile {
public:
    explicit RaceProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Race; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    uint8_t get_profile_id() override { return PROFILE_ID_RACE; }
};

extern AgilityProfile* agility;
extern ComfortProfile* comfort;
extern WinterProfile* winter;
extern ManualProfile* manual;
extern StandardProfile* standard;
extern RaceProfile* race;

#endif