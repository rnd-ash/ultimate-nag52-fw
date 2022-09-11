// Profile code

#ifndef __PROFILES_H_
#define __PROFILES_H_

#include "canbus/can_hal.h"
#include "common_structs.h"
#include "tcm_maths.h"

#define MAX_PROFILES 4

#define PROFILE_ID_STANDARD 0
#define PROFILE_ID_COMFORT 1
#define PROFILE_ID_WINTER 2
#define PROFILE_ID_AGILITY 3
#define PROFILE_ID_MANUAL 4

#define SHIFT_MAP_X_SIZE 11
#define SHIFT_MAP_Y_SIZE 4
const int16_t shift_table_x_header[SHIFT_MAP_X_SIZE] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
const int16_t upshift_y_headers[SHIFT_MAP_Y_SIZE] = {1,2,3,4};
const int16_t downshift_y_headers[SHIFT_MAP_Y_SIZE] = {2,3,4,5};


/**
 * A profile is designed to read the current conditions of the gearbox and request the gearbox to do something
 */
class AbstractProfile {
public:
    AbstractProfile(
        bool is_diesel, 
        const char* tag_id, 
        const char* upshift_map_name_diesel, 
        const char* downshift_map_name_diesel, 
        const char* upshift_map_name_petrol, 
        const char* downshift_map_name_petrol,
        const int16_t* def_upshift_data_diesel,
        const int16_t* def_downshift_data_diesel,
        const int16_t* def_upshift_data_petrol,
        const int16_t* def_downshift_data_petrol
    );
    virtual GearboxProfile get_profile() const = 0;
    virtual GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) = 0;
    virtual bool should_upshift(GearboxGear current_gear, SensorData* sensors) = 0;
    virtual bool should_downshift(GearboxGear current_gear, SensorData* sensors) = 0;
    virtual ShiftCharacteristics get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) = 0;
    virtual TccLockupBounds get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) = 0;

    TcmMap* get_upshift_map() {
        return this->upshift_table;
    }
    TcmMap* get_downshift_map() {
        return this->downshift_table;
    }
    virtual GearboxGear get_start_gear() const {
        return GearboxGear::First;
    }
    void increment_subprofile() {
        profile_id += 1;
        if (profile_id >= MAX_PROFILES) {
            profile_id = 0;
        }
    }
    virtual uint8_t get_profile_id() = 0;
    void reload_data();
protected:
    uint8_t profile_id = 0;
    TcmMap* upshift_table = nullptr;
    TcmMap* downshift_table = nullptr;
private:
    bool is_diesel; 
    const char* tag_id; 
    const char* upshift_map_name_diesel; 
    const char* downshift_map_name_diesel; 
    const char* upshift_map_name_petrol;
    const char* downshift_map_name_petrol;
    const int16_t* def_upshift_data_diesel;
    const int16_t* def_downshift_data_diesel;
    const int16_t* def_upshift_data_petrol;
    const int16_t* def_downshift_data_petrol;
};

class AgilityProfile : public AbstractProfile {
public:
    AgilityProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Agility; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    ShiftCharacteristics get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) override;
    TccLockupBounds get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) override;
    uint8_t get_profile_id() override { return PROFILE_ID_AGILITY; }
};

class ComfortProfile : public AbstractProfile {
public:
    ComfortProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Comfort; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    ShiftCharacteristics get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) override;
    TccLockupBounds get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) override;
    uint8_t get_profile_id() override { return PROFILE_ID_COMFORT; }
    GearboxGear get_start_gear() const override {
        return GearboxGear::Second;
    }
};

class WinterProfile : public AbstractProfile {
public:
    WinterProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Winter; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    ShiftCharacteristics get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) override;
    TccLockupBounds get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) override;
    uint8_t get_profile_id() override { return PROFILE_ID_WINTER; }
    GearboxGear get_start_gear() const override {
        return GearboxGear::Second;
    }
};

class StandardProfile : public AbstractProfile {
public:
    StandardProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Standard; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    TccLockupBounds get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) override;
    ShiftCharacteristics get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) override;
    uint8_t get_profile_id() override { return PROFILE_ID_STANDARD; }
};

class ManualProfile : public AbstractProfile {
public:
    ManualProfile(bool is_diesel);
    GearboxProfile get_profile() const override { return GearboxProfile::Manual; }
    GearboxDisplayGear get_display_gear(GearboxGear target, GearboxGear actual) override;
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    TccLockupBounds get_tcc_lockup_bounds(SensorData* sensors, GearboxGear curr_gear) override;
    ShiftCharacteristics get_shift_characteristics(ProfileGearChange requested, SensorData* sensors) override;
    uint8_t get_profile_id() override { return PROFILE_ID_MANUAL; }
};

extern AgilityProfile* agility;
extern ComfortProfile* comfort;
extern WinterProfile* winter;
extern ManualProfile* manual;
extern StandardProfile* standard;

#endif