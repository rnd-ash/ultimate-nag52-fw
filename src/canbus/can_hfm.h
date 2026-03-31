#ifndef HFM_CAN_H
#define HFM_CAN_H

#include "can_hal.h"
#include "../../lib/hfm/src/HFM.h"
#include "shifter/shifter.h"
#include "stored_table.h"
#include "maps.h"
#include "nvs/all_keys.h"

#include "shifter/shifter_trrs.h"

/// @brief The CAN-layer supports classic Mercedes-Benz cars from the early 1990's (e.g. 124, 202, 129, 140, 210) with a 125kbit/s CAN for HFM coming partially with M104 (L6) and M111 (L4). It does not support the 500kbit/s CAN coming with the M119 (V8) in cars of the same era.
class HfmCan: public EgsBaseCan {
    public:
        HfmCan(const char* name, uint8_t tx_time_ms);

        /**
         * Getters
         */

        // Get the front right wheel data
        uint16_t get_front_right_wheel(const uint32_t expire_time_ms)  override;
        // Get the front left wheel data
        uint16_t get_front_left_wheel(const uint32_t expire_time_ms) override;
        // Get the rear right wheel data
        uint16_t get_rear_right_wheel(const uint32_t expire_time_ms) override;
        // Get the rear left wheel data
        uint16_t get_rear_left_wheel(const uint32_t expire_time_ms) override;
        // Gets engine type
        EngineType get_engine_type(const uint32_t expire_time_ms) override;
        // Returns true if engine is in limp mode
        bool get_engine_is_limp(const uint32_t expire_time_ms) override;
        // Returns true if pedal is kickdown 
         bool get_kickdown(const uint32_t expire_time_ms) override;
        // Returns the pedal percentage. Range 0-250
         uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        // Gets Torque information
        CanTorqueData get_torque_data(const uint32_t expire_time_ms) override;
        float get_ML(const uint32_t expire_time_ms);
        // Gets engine coolant temperature
        int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) override;
        // Gets engine oil temperature
         int16_t get_engine_oil_temp(const uint32_t expire_time_ms) override;
         // Gets engine charge air temperature
        int16_t get_engine_iat_temp(const uint32_t expire_time_ms) override;
        // Gets engine RPM
         uint16_t get_engine_rpm(const uint32_t expire_time_ms) override;
        // Returns true if engine is cranking
        bool get_is_starting(const uint32_t expire_time_ms) override;
        // 
        bool get_is_brake_pressed(const uint32_t expire_time_ms) override;

        /**
         * Setters
         */
        // Set the actual gear of the gearbox
        void set_actual_gear(GearboxGear actual) override;
        // Set the target gear of the gearbox
        void set_target_gear(GearboxGear target) override;
        // Sets torque request toggle
        void set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) override;
        void set_drive_profile(GearboxProfile p) override;

    protected:
        void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) override;

    private:
        
        // [°C] temperature of intake air
        const int16_t INTAKE_AIR_TEMPERATURE[256] = {70, 70, 70, 70, 70, 70, 130, 125, 120, 117, 113, 110, 107, 103, 100, 98, 96, 94, 92, 90, 89, 87, 86, 84, 83, 81, 80, 79, 78, 77, 76, 74, 73, 72, 71, 70, 69, 68, 68, 67, 66, 65, 64, 63, 63, 62, 61, 60, 59, 59, 58, 58, 57, 56, 56, 55, 54, 54, 53, 53, 52, 51, 51, 50, 50, 49, 49, 48, 48, 47, 47, 46, 46, 45, 45, 44, 44, 43, 43, 42, 42, 41, 41, 40, 40, 39, 39, 38, 38, 38, 37, 37, 36, 36, 35, 35, 35, 34, 34, 33, 33, 33, 32, 32, 31, 31, 30, 30, 30, 29, 29, 29, 28, 28, 28, 27, 27, 26, 26, 26, 25, 25, 25, 24, 24, 24, 23, 23, 23, 22, 22, 21, 21, 21, 20, 20, 20, 19, 19, 19, 18, 18, 18, 17, 17, 17, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -7, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -13, -13, -14, -14, -15, -16, -16, -17, -18, -18, -19, -19, -20, -21, -22, -23, -24, -25, -26, -26, -27, -28, -29, -30, -31, -33, -34, -36, -37, -39, -40, 70, 70, 70, 70, 70, 70, 70, 70};

        // [°C] temperature of coolant
        const int16_t COOLANT_TEMPERATURE[256] = {107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 107, 136, 134, 131, 129, 127, 124, 122, 121, 119, 117, 115, 114, 112, 111, 109, 108, 106, 105, 104, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 85, 84, 83, 82, 81, 81, 80, 79, 78, 78, 77, 76, 76, 75, 74, 73, 73, 72, 72, 71, 70, 70, 69, 68, 68, 67, 67, 66, 66, 65, 64, 64, 63, 63, 62, 62, 61, 61, 60, 59, 59, 58, 58, 57, 57, 56, 56, 55, 55, 54, 54, 53, 53, 52, 52, 52, 51, 51, 50, 50, 49, 49, 48, 48, 47, 47, 46, 46, 46, 45, 45, 44, 44, 43, 43, 42, 42, 42, 41, 41, 40, 40, 39, 39, 39, 38, 38, 37, 37, 36, 36, 35, 35, 35, 34, 34, 33, 33, 32, 32, 32, 31, 31, 30, 30, 29, 29, 29, 28, 28, 27, 27, 26, 26, 26, 25, 25, 24, 24, 23, 23, 22, 22, 21, 21, 21, 20, 20, 19, 19, 18, 18, 17, 17, 16, 16, 15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 6, 5, 4, 4, 3, 3, 2, 1, 1, 0, -1, -1, -2, -3, -4, -4, -5, -6, -7, -8, -8, -9, -10, -11, -12, -13, -14, -15, -17, -18, -19, -20, -22, -23, -25, -26, -28, -30, -33, -35, -38, -42, -46, -52, -61, -71};

        const float AIR_MASS_FACTOR = 4.F; // conversion factor for raw-value from CAN
        int16_t ALTITUDE_PRESSURE_FACTOR = 5; // conversion factor for raw-value from CAN

        // [Nm] typical drag torque for an air conditioning compressor in a combustion engine car
        // is in the range of 10-30 Nm depending on compressor type and operating conditions.
        const int16_t M_KOMP = 20;
        
        // [-] limit for compatibility with other CAN-layers
        const int32_t PEDAL_VALUE_LIMIT = 250; 

        const float a   = 20.F; // Torque loss at 0 rpm in Nm
        const float b   = 5.5F; // Linear coefficient for rpm-dependent torque loss in Nm/(1000rpm)
        const float c   = 1.1F; // Quadratic coefficient for rpm-dependent torque loss
        
        const int16_t K_AC = 12000;

        const int16_t K_GENERATOR = 8000;

        // [kg/m³] air density
        float current_air_pressure = 0.0f;
        // [°C] intake air temperature
        float current_intake_air_temp = 0.0f;

        ECU_HFM hfm_ecu = ECU_HFM();
        
        bool start_enable = false;

        bool freeze_torque = false;
        int16_t req_static_torque_delta = 0;

        GearboxGear actual_gear =  GearboxGear::SignalNotAvailable;
        GearboxGear target_gear = GearboxGear::SignalNotAvailable;

        // limit the maximum torque by gearbox request
        bool MMAX_EGS = false;

        uint16_t generateWheelData(const uint32_t expire_time_ms) const;
};

#endif // HFM_CAN_H