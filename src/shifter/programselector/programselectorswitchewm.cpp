#include "programselectorswitchewm.h"
#include "../../clock.hpp"
#include "../../nvs/module_settings.h"

ProgramSelectorSwitchEWM::ProgramSelectorSwitchEWM()
{
}

AbstractProfile *ProgramSelectorSwitchEWM::get_profile(const uint32_t expire_time_ms)
{
	AbstractProfile *result = nullptr;
	ProfileSwitchPos profileswitchpos = is_pressed ? ProfileSwitchPos::Top : ProfileSwitchPos::Bottom;
	GearboxProfile profile = ETS_CURRENT_SETTINGS.profile_idx_top;
	if (ProfileSwitchPos::Bottom == profileswitchpos)
	{
		profile = ETS_CURRENT_SETTINGS.profile_idx_bottom;
	}
	result = profiles[profile];
	return result;
}
