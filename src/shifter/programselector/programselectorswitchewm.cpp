#include "programselectorswitchewm.h"
#include "../../clock.hpp"
#include "../../nvs/module_settings.h"

ProgramSelectorSwitchEWM::ProgramSelectorSwitchEWM()
{
}

void ProgramSelectorSwitchEWM::set_profile_switch_pos(ProfileSwitchPos pos) {
	this->pos = pos;
}

AbstractProfile *ProgramSelectorSwitchEWM::get_profile(const uint32_t expire_time_ms)
{
	AbstractProfile *result = nullptr;
	GearboxProfile profile = ETS_CURRENT_SETTINGS.profile_idx_top;
	if (ProfileSwitchPos::Bottom == this->pos)
	{
		profile = ETS_CURRENT_SETTINGS.profile_idx_bottom;
	}
	result = profiles[profile];
	return result;
}

DiagProfileInputState ProgramSelectorSwitchEWM::get_input_raw() const {
	DiagProfileInputState pos = DiagProfileInputState::SNV;
	switch (this->pos) {
		case ProfileSwitchPos::Top:
			pos = DiagProfileInputState::SwitchTop;
			break;
		case ProfileSwitchPos::Bottom:
			pos = DiagProfileInputState::SwitchBottom;
			break;
		case ProfileSwitchPos::SNV:
		default:
			break;
	}
	return pos;
}

ProgramSelectorType ProgramSelectorSwitchEWM::get_type() const {
	return ProgramSelectorType::EWMSwitch;
}
