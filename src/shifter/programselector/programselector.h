#ifndef PROGRAMSELECTOR_H
#define PROGRAMSELECTOR_H

#include "../../profiles.h"

enum class ProgramSelectorType {
	SLR,
	EWMButton,
	EWMSwitch,
	TRRS
};

/**
 * @brief used for TRRS profile switch and EWM profile switch position
 */
enum class ProfileSwitchPos : uint8_t
{
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
 * @brief For diagnostics only, an enum with all possible profile input states
 * This is used for capturing raw inputs on the shifter
 */
enum class DiagProfileInputState: uint8_t {
	None = 0,
	SwitchTop = 1,
	SwitchBottom = 2,
	ButtonPressed = 3,
	ButtonReleased = 4,
	SLRLeft = 5,
	SLRMiddle = 6,
	SLRRight = 7,
	/**
	 * @brief Could not be determined (Comm error)
	 */
	SNV = UINT8_MAX
};

class ProgramSelector {
public:
	/**
	 * @brief Gets the current selected drive profile based on the profile selector input system
	 * @param expire_time_ms Expiry time of CAN data
	 * @return Profile that should be in use. nullptr is returned if the current
	 * switch position could not be determined, in which case, we do not switch profiles.
	 */
	virtual AbstractProfile* get_profile(const uint32_t expire_time_ms);
	/**
	 * @brief For identification
	 * @return 
	 */
	virtual ProgramSelectorType get_type() const;

	virtual DiagProfileInputState get_input_raw() const {
		return DiagProfileInputState::None;
	};
protected:
	uint8_t profile_id = 0u;	
};

#endif