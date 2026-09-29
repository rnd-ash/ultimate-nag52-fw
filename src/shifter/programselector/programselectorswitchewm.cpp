#include "programselectorswitchewm.h"
#include "../../clock.hpp"
#include "../../nvs/module_settings.h"

ProgramSelectorSwitchEWM::ProgramSelectorSwitchEWM()
{
}

void ProgramSelectorSwitchEWM::set_profile_switch_pos(ProfileSwitchPos pos) {
	this->pos = pos;
}

AbstractProfile *ProgramSelectorSwitchEWM::get_profile(void)
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
	DiagProfileInputState position = DiagProfileInputState::SNV;
	switch (this->pos) {
		case ProfileSwitchPos::Top:
			position = DiagProfileInputState::SwitchTop;
			break;
		case ProfileSwitchPos::Bottom:
			position = DiagProfileInputState::SwitchBottom;
			break;
		case ProfileSwitchPos::SNV:
		default:
			break;
	}
	return position;
}

ProgramSelectorType ProgramSelectorSwitchEWM::get_type() const {
	return ProgramSelectorType::EWMSwitch;
}
