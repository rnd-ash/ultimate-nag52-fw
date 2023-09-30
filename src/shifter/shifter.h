#ifndef SHIFTER_H
#define SHIFTER_H

#include <stdint.h>
#include "clock.hpp"

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
 * @brief TRRS profile switch position
 */
enum class ProfileSwitchPos : uint8_t {
    /**
     * @brief Top position (S)
     */
    Top = 0u,
    /**
     * @brief Bottom position (C or W)
     */
    Bottom = 1u,
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

    /**
	 * @brief Gets the position of the W/S switch on the shifter.
     * NOTE: This function exists for both TRRS and EWM shifter types as it was originally
     * designed for TRRS only, but Chrysler EWM ECUs actually have a W/S Switch bit on CAN,
     * so therefore, this function applies to both shifter types.
     * 
	 * @param expire_time_ms data expiration period
	 * @return Current profile switch position
	 */
    virtual ProfileSwitchPos get_shifter_profile_switch_pos(const uint32_t expire_time_ms) = 0;
};

#endif // SHIFTER_H