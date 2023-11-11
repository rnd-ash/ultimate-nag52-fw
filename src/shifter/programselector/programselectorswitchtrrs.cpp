#include "programselectorswitchtrrs.h"
#include "../../nvs/module_settings.h"

ProgramSelectorSwitchTRRS::ProgramSelectorSwitchTRRS(BoardGpioMatrix *board, AbstractProfile *profiles): board(board) {
	this->profiles = profiles;
}

AbstractProfile *ProgramSelectorSwitchTRRS::get_profile(const uint32_t expire_time_ms)
{
	AbstractProfile *result = nullptr;
	if (nullptr != profiles)
	{
		result = &profiles[GearboxProfile::Underscore];
		if ((nullptr != board) && board->is_data_valid(expire_time_ms))
		{
			ProfileSwitchPos profileswitchpos;
			bool tmp = board->is_program_switch_pressed();
			profileswitchpos = tmp ? ProfileSwitchPos::Top : ProfileSwitchPos::Bottom;
			GearboxProfile profile = ETS_CURRENT_SETTINGS.profile_idx_top;
			if (ProfileSwitchPos::Bottom == profileswitchpos)
			{
				profile = ETS_CURRENT_SETTINGS.profile_idx_bottom;
			}
			result = &profiles[profile];
		}
	}
	return result;
}