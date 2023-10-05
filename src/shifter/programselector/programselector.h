#ifndef PROGRAMSELECTOR_H
#define PROGRAMSELECTOR_H

#include "../../profiles.h"

class ProgramSelector {
public:
	virtual AbstractProfile* get_profile(const uint32_t expire_time_ms);
protected:
	AbstractProfile* profiles;
	uint8_t profile_id = 0u;	
};

#endif