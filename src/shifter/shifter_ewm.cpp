#include "shifter_ewm.h"
#include "programselector/programselectorbutton.h"
#include "programselector/programselectorswitchewm.h"
#include "programselector/programselectorSLR.h"

ShifterEwm::ShifterEwm(TCM_CORE_CONFIG *vehicle_config, ETS_MODULE_SETTINGS *shifter_settings)
{
	this->vehicle_config = vehicle_config;
	if (((uint8_t)ShifterStyle::SLR) != vehicle_config->shifter_style)
	{
		switch (shifter_settings->ewm_selector_type)
		{
		case EwmSelectorType::Switch:
			programselector = new ProgramSelectorSwitchEWM();
			break;
		case EwmSelectorType::Button:
			programselector = new ProgramSelectorButton(vehicle_config);
			break;
		default:
			programselector = nullptr;
			break;
		}
	}
	else
	{
		programselector = new ProgramSelectorSLR(pcb_gpio_matrix);
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
		(reinterpret_cast<ProgramSelectorButton *>(programselector))->set_button_pressed(is_pressed);
	}
}
