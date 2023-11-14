#ifndef PROGRAMSELECTOR_H
#define PROGRAMSELECTOR_H

#include "../../profiles.h"

class ProgramSelector {
public:
	/**
	 * @brief Gets the current selected drive profile based on the profile selector input system
	 * @param expire_time_ms Expiry time of CAN data
	 * @return Profile that should be in use. nullptr is returned if the current
	 * switch position could not be determined, in which case, we do not switch profiles.
	 */
	virtual AbstractProfile* get_profile(const uint32_t expire_time_ms);
protected:
	AbstractProfile** profiles;
	uint8_t profile_id = 0u;	
};

#endif