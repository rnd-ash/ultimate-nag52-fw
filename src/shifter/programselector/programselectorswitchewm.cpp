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
	GearboxProfile profile = selectableProfileToProfile(ETS_CURRENT_SETTINGS.switch_profile_idx_top);
	if (ProfileSwitchPos::Bottom == this->pos)
	{
		profile = selectableProfileToProfile(ETS_CURRENT_SETTINGS.switch_profile_idx_bottom);
	}
	result = profiles[(uint8_t)profile];
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
