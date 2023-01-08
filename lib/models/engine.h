#ifndef ENGINE_H
#define ENGINE_H

#include <stdint.h>
#include "../core/lookuptable.h"

static const float DIAMETER_TO_RADIUS_FACTOR = 0.5F;

/// @brief This class allows the calculation of the engine's torques out of other measured values. 
/// @brief It is required for classic Mercedes-Benz vehicles from the early 1990's, where some measured values where available on the CAN, but not the torque value required by the TCU. Cars with custom ECUs might also make use of this.
class Engine {

public:
    /// @brief Initializes the engine model.
    /// @param c constant required to convert mdot and engine speed to the engine torque (in [m²/s²])
    /// @param _M_eng_max Lookuptable with the maximum engine torques
    /// @param throttle_diameter diameter of the trottle (in [m])
    Engine(const float c, const LookupTable* _M_eng_max, const float throttle_diameter);
    
    /// @brief Frees the memory
    ~Engine();

    /// @brief Contains a set of torques required by the TCU to shift.
    struct Torques{

        /// @brief minimum engine torque (in [Nm])
        uint16_t minTorque;

        /// @brief maximum engine torque (in [Nm])
        uint16_t maxTorque;
        
        /// @brief static engine torque (in [Nm])
        uint16_t staticTorque;

        /// @brief (driver) demanded engine torque (in [Nm])
        uint16_t demandedTorque;
    };

    /// @brief Calculates the engine torque based on the current status of the engine
    /// @param engine_speed the engine speed (in [rpm])
    /// @param throttle_valve_opening_angle the opening angle of the throttle valve (in [°])
    /// @param p_a pressure before the throttle valve (in [hPa])
    /// @param T_a outside temperature (in [°C])
    /// @param maf mass air flow (in [kg/h])
    /// @return A set with the calculated torques
    Torques getTorques(const uint16_t engine_speed, const float throttle_valve_opening_angle, const uint16_t p_a, , const uint16_t T_a, const uint16_t maf);

private:
    /// @brief radius of the throttle valve (in [m])
    float r;

    /// @brief constant for converting mdot and engine speed to the engine torque (in [m²/s²])
    float c_eng;

    /// @brief cross-sectional area of the throttle valve (in [m²])
    float A_throttle_valve;
    
    /// @brief lookup table with the maximum engine torques
    LookupTable* M_eng_max;

    /// @brief Calculates the engine torque based on engine speed, mass air flow, and engine constant.
    /// @param engine_speed the engine speed (in [rpm])
    /// @param maf mass air flow (in [kg/h])
    /// @return static engine torque based on current engine speed, mass air flow, and engine constant (in [Nm]) 
    uint16_t calculateTorque(const uint16_t engine_speed, const uint16_t maf);

    /// @brief Estimates the demanded engine torque.
    /// @param engine_speed the engine speed (in [rpm])
    /// @param throttle_valve_opening_angle the flare angle of the throttle valve (in [°])
    /// @param p_a pressure before the throttle valve (in [hPa])
    /// @param T_a outside temperature (in [°C])
    /// @return demanded engine torque based on current engine speed, estimated mass air flow, engine constants, and throttle valve opening angle
    uint16_t estimateDemandedTorque(const uint16_t engine_speed, const float throttle_valve_opening_angle, const uint16_t p_a, const uint16_t T_a);

    float psi(const uint16_t p_a, const uint16_t p_s);

};
#endif /* ENGINE_H */