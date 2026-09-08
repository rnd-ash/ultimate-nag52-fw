#include "shifter_ewm.h"
#include "programselector/programselectorbuttonewm.h"
#include "programselector/programselectorswitchewm.h"
#include "programselector/programselectorSLR.h"

ShifterEwm::ShifterEwm(const ETS_MODULE_SETTINGS *shifter_settings)
{
	if (((uint8_t)ShifterStyle::SLR) != VEHICLE_CONFIG.shifter_style)
	{
		switch (shifter_settings->ewm_selector_type)
		{
		case EwmSelectorType::Switch:
			programselector = new ProgramSelectorSwitchEWM();
			break;
		case EwmSelectorType::Button:
			programselector = new ProgramSelectorButtonEwm();
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

DiagProfileInputState ShifterEwm::diag_get_profile_input(void) {
	// None rather than SNV (SNV means valid configuration, but no communication)
	// None implied not configured / no program selector
	DiagProfileInputState ret = DiagProfileInputState::None;
	if (nullptr != this->programselector) {
		ret = this->programselector->get_input_raw();
	}
	return ret;
}

ShifterPosition ShifterEwm::get_shifter_position(void)
{
	ShifterPosition pos = ShifterPosition::SignalNotAvailable;
	if (nullptr != egs_can_hal) {

		pos = egs_can_hal->internal_can_shifter_get_shifter_position(expire_time_ms);
	}
	return pos;
}

AbstractProfile *ShifterEwm::get_profile(void)
{
	AbstractProfile *result = nullptr;
	if (nullptr != programselector)
	{
		result = programselector->get_profile();
	} else {
		// null selector can be if the selector has no profile button (Jeep/Sprinter)
		result = profiles[VEHICLE_CONFIG.default_profile];
	}
	return result;
}

void ShifterEwm::set_program_button_pressed(const bool is_pressed, const ProfileSwitchPos pos)
{
	if (nullptr != programselector)
	{
		switch (programselector->get_type()) {
			case ProgramSelectorType::EWMButton:
				(reinterpret_cast<ProgramSelectorButtonEwm *>(programselector))->set_button_pressed(is_pressed);
				break;
			case ProgramSelectorType::EWMSwitch:
				(reinterpret_cast<ProgramSelectorSwitchEWM *>(programselector))->set_profile_switch_pos(pos);
				break;
			default:
				break;
		}
	}
}

ShifterStyle ShifterEwm::get_shifter_type() {
	return ShifterStyle::EWM;
}

void ShifterEwm::update(void)
{
	set_program_button_pressed(egs_can_hal->get_profile_btn_press(expire_time_ms), egs_can_hal->get_profile_switch_pos(expire_time_ms));
}
