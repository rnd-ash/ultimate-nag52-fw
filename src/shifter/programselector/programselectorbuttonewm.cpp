#include "programselectorbuttonewm.h"


ProgramSelectorButtonEwm::ProgramSelectorButtonEwm(TCM_CORE_CONFIG *vehicle_config): vehicle_config(vehicle_config)
{
	// Read profile ID on startup based on TCM config
	profile_id = VEHICLE_CONFIG.default_profile;
	if (profile_id > 4)
	{
		profile_id = 0;
	}
}

void ProgramSelectorButtonEwm::set_button_pressed(const bool is_pressed)
{
	if(is_pressed && (is_pressed != is_pressed_last_call)) {
		// button is pressed and was not pressed before
		// TODO: add logic to only allow certain profiles
		profile_id++;
		if (profile_id == NUM_PROFILES)
		{
			profile_id = 0u;
		}
	}
	is_pressed_last_call = is_pressed;
}

AbstractProfile *ProgramSelectorButtonEwm::get_profile(const uint32_t expire_time_ms)
{
	return profiles[profile_id];
}

ProgramSelectorType ProgramSelectorButtonEwm::get_type() const {
	return ProgramSelectorType::EWMButton;
}
