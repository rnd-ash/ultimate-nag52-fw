#ifndef ENGINE_H
#define ENGINE_H

#include <stdint.h>

static const float DIAMETER_TO_RADIUS_FACTOR = 0.5F;

/// @brief This class allows the calculation of the engine's torques out of other measured values. 
/// @brief It is required for classic Mercedes-Benz vehicles from the early 1990's, where some measured values where available on the CAN, but not the torque value required by the TCU. Cars with custom ECUs might also make use of this.
class Engine {

public:
    /// @brief Initializes the engine model.
    /// @param c constant required to convert mdot and current engine speed to the engine static torque (in [m²/s²])
    /// @param M_eng_max array with the engine's maximum torque values (at wide open throttle, in [Nm])
    /// @param M_eng_max_headers array with headers for the engine maximum torque values (in [rpm])
    /// @param length_M_eng_max length of the array with the engines maximum torque values
    /// @param throttle_diameter diameter of the trottle (in [m])
    Engine(float c, uint16_t* M_eng_max, uint16_t* M_eng_max_headers, uint16_t length_M_eng_max, float throttle_diameter);
    
    /// @brief 
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
    /// @param throttle_valve_flare_angle the flare angle of the throttle valve (in [°])
    /// @param p_a pressure before the throttle valve (in [hPa])
    /// @param maf mass air flow (in [kg/h])
    /// @return A set with the calculated torques
    Torques getTorques(uint16_t engine_speed, float throttle_valve_flare_angle, uint16_t p_a, uint16_t maf);

private:
    /// @brief radius of the throttle valve (in [m])
    float r;

    /// @brief 
    float c_eng;

    /// @brief cross-sectional area of the throttle valve (in [m²])
    float A_throttle_valve;
    
    /// @brief array with the engine maximum torque values (at wide open throttle, in [Nm])
    uint16_t* M_eng_max_loc;
    
    /// @brief array with headers for the engine maximum torque values (in [rpm])
    uint16_t* M_eng_max_headers_loc;

    /// @brief length of the array with the engines maximum torque values
    uint16_t length_M_eng_max_loc;

    /// @brief determines, if the allocation of memory for the arrays was successful
    bool allocation_succeeded;
};
#endif /* ENGINE_H */