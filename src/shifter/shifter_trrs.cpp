#include "shifter_trrs.h"
#include "ioexpander.h"

ShifterTrrs::ShifterTrrs(esp_err_t *can_init_status)
{
	*can_init_status = ioexpander->init_state();
}

ShifterPosition ShifterTrrs::get_shifter_position(const uint32_t expire_time_ms)
{
	ShifterPosition result = ShifterPosition::SignalNotAvailable;
	if (ioexpander->is_data_valid(expire_time_ms))
	{
		uint8_t trrs = ioexpander->get_trrs();
		if (16u > trrs)
		{
			if (0u < trrs)
			{
				// check truth table
				result = TRRS_SHIFTER_TABLE[trrs];
			}
			else
			{
				// intermediate position, now work out which one
				switch (this->last_valid_position)
				{
				case ShifterPosition::P:
					result = ShifterPosition::P_R;
					break;
				case ShifterPosition::R:
					result = ShifterPosition::R_N;
					break;
				case ShifterPosition::D:
					result = ShifterPosition::N_D;
					break;
				case ShifterPosition::N:
					result = ShifterPosition::N_D;
					break;
				default:
					result = ShifterPosition::SignalNotAvailable;
					break;
				}
			}
		}
	}
	bool start_enable = (ShifterPosition::P == result || ShifterPosition::N == result);
	ioexpander->set_start(start_enable);
	return result;
}

ProfileSwitchPos ShifterTrrs::get_shifter_profile_switch_pos(const uint32_t expire_time_ms) {
	ProfileSwitchPos result = ProfileSwitchPos::SNV;
	if(ioexpander->is_data_valid(expire_time_ms)){
		result = ioexpander->get_program_switch();
	}
	return result;
}

void ShifterTrrs::set_rp_solenoid(const float vVeh, const ShifterPosition pos, const bool is_brake_pressed)
{
	bool should_rp_solenoid_be_activated = (ShifterPosition::N == pos) && ((2.5F < vVeh) || is_brake_pressed);
	ioexpander->set_rp_solenoid(should_rp_solenoid_be_activated);
}
