#ifndef SHIFTER_H
#define SHIFTER_H

#include <stdint.h>
#include "clock.hpp"
#include "programselector/programselector.h"
#include "../nvs/eeprom_config.h"

/**
 * @brief Gear shifter style
 */
enum class ShifterStyle : uint8_t {
    /**
     * @brief EWM TipTronic CAN based shifter (has +/-)
     */
    EWM = 0u,
	/**
	 * @brief TRRS hard-wired shifter (Has 4/3/2/1)
	 */
	TRRS = 1u,
	/**
	 * @brief SLR Mclaran shifter
     * NOTE: This is unimplemented at the moment.
	 */
	SLR = 2u,
};


/**
 * @brief SLR profile knob position
 */
enum class SLRProfileWheel : uint8_t {
    /**
     * @brief Right (S)
     */
    Right = 0u,
    /**
     * @brief Center (M)
     */
    Center = 1u,
    /**
     * @brief Left (C)
     */
    Left = 2u,
    /**
     * @brief Could not determine position
     */
    SNV = UINT8_MAX,
};

/**
 * @brief Gear selector position
 */
enum class ShifterPosition : uint8_t {
    /**
     * @brief Park
     */
    P,

    /**
     * @brief Between Park and Reverse
     */
    P_R,

    /**
     * @brief Reverse
     */
    R,

    /**
     * @brief Between Reverse and Neutral
     */
    R_N,

    /**
     * @brief Neutral
     */
    N,

    /**
     * @brief Between Neutral and Drive
     */
    N_D,

    /**
     * @brief Drive
     */
    D,

    /**
     * @brief EWM Shifter only - TipTronic Plus position
     */
    PLUS,

    /**
     * @brief EWM Shifter only - TipTronic Minus position
     */
    MINUS,

    /**
     * @brief TRRS Shifter only - Range restrict position '4'
     */
    FOUR,

    /**
     * @brief TRRS Shifter only - Range restrict position '3'
     */
    THREE,

    /**
     * @brief TRRS Shifter only - Range restrict position '2'
     */
    TWO,

    /**
     * @brief TRRS Shifter only - Range restrict position '1'
     */
    ONE,

    /**
     * @brief Could not determine shifter position
     */
    SignalNotAvailable = UINT8_MAX, // SNV
};

class Shifter
{
public:
	/**
	 * @brief Gets the current gear selector position
	 * @param expire_time_ms data expiration period
	 * @return Current gear selector position
	 */
	virtual ShifterPosition get_shifter_position(const uint32_t expire_time_ms) = 0;

    virtual AbstractProfile* get_profile(const uint32_t expire_time_ms) = 0;

    void set_shifter_position(ShifterPosition spos);
    void set_brake_is_pressed(bool is_pressed);
    void set_vehicle_speed(WheelData front_left, WheelData front_right);
    
protected:
    TCM_CORE_CONFIG *vehicle_config;    
    ShifterPosition spos = ShifterPosition::SignalNotAvailable;
    bool is_brake_pressed = false;
    float vVeh = 0.0F;
};

#endif // SHIFTER_H