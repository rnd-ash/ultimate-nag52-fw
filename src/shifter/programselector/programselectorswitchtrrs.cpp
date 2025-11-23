#include "programselectorswitchtrrs.h"
#include "../../nvs/module_settings.h"

ProgramSelectorSwitchTRRS::ProgramSelectorSwitchTRRS(BoardGpioMatrix *board): board(board) {
}

AbstractProfile *ProgramSelectorSwitchTRRS::get_profile(void)
{
	AbstractProfile *result = nullptr;
	if ((nullptr != board) && board->is_data_valid(expire_time_IC_query))
	{
		ProfileSwitchPos profileswitchpos;
		bool tmp = board->is_program_switch_pressed();
		profileswitchpos = tmp ? ProfileSwitchPos::Top : ProfileSwitchPos::Bottom;
		GearboxProfile profile = selectableProfileToProfile(ETS_CURRENT_SETTINGS.switch_profile_idx_top);
		if (ProfileSwitchPos::Bottom == profileswitchpos)
		{
			profile = selectableProfileToProfile(ETS_CURRENT_SETTINGS.switch_profile_idx_bottom);
		}
		result = profiles[(uint8_t)profile];
	}
	return result;
}

DiagProfileInputState ProgramSelectorSwitchTRRS::get_input_raw() const {
	DiagProfileInputState pos = DiagProfileInputState::SNV;
	if ((nullptr != board) && board->is_data_valid(expire_time_IC_query)) {
		pos = board->is_program_switch_pressed() ? DiagProfileInputState::SwitchTop : DiagProfileInputState::SwitchBottom;
	}
	return pos;
}

ProgramSelectorType ProgramSelectorSwitchTRRS::get_type() const {
	return ProgramSelectorType::TRRS;
}