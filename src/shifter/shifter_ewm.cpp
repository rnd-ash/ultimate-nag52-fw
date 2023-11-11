#include "shifter_ewm.h"
#include "programselector/programselectorbutton.h"
#include "programselector/programselectorswitchewm.h"

ShifterEwm::ShifterEwm(TCM_CORE_CONFIG *vehicle_config, const ETS_MODULE_SETTINGS *shifter_settings, AbstractProfile *profiles)
{
	this->vehicle_config = vehicle_config;
	if (((uint8_t)ShifterStyle::SLR) != vehicle_config->shifter_style)
	{
		switch (shifter_settings->ewm_selector_type)
		{
		case EwmSelectorType::Switch:
			programselector = new ProgramSelectorSwitchEWM(profiles);
			break;
		case EwmSelectorType::Button:
			programselector = new ProgramSelectorButton(vehicle_config, profiles);
			break;
		default:
			programselector = nullptr;
			break;
		}
	}
	else
	{
		// TODO: implement ProgramSelectorSLR;
		// programselector = new ProgramSelectorSLR(profiles);
	}
}

ShifterPosition ShifterEwm::get_shifter_position(const uint32_t expire_time_ms)
{
	return spos;
}

AbstractProfile *ShifterEwm::get_profile(const uint32_t expire_time_ms)
{
	AbstractProfile *result = nullptr;
	if (nullptr != programselector)
	{
		result = programselector->get_profile(expire_time_ms);
	}
	return result;
}

void ShifterEwm::set_program_button_pressed(const bool is_pressed)
{
	if (nullptr != programselector)
	{
		(reinterpret_cast<ProgramSelectorEWM *>(programselector))->set_button_pressed(is_pressed);
	}
}
