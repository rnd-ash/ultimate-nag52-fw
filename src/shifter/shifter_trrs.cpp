#include "shifter_trrs.h"
#include "../profiles.h"
#include "nvs/module_settings.h"
#include "programselector/programselectorswitchtrrs.h"
#include "canbus/can_hal.h"
#include "inputcomponents/brakepedal.hpp"

ShifterTrrs::ShifterTrrs(TCM_CORE_CONFIG *vehicle_config, BoardGpioMatrix *board): board(board)
{
	this->vehicle_config = vehicle_config;
	this->programselector = new ProgramSelectorSwitchTRRS(board);
}

DiagProfileInputState ShifterTrrs::diag_get_profile_input() {
	// None rather than SNV (SNV means valid configuration, but no communication)
	// None implied not configured / no program selector
	DiagProfileInputState ret = DiagProfileInputState::None;
	if (nullptr != this->programselector) {
		ret = this->programselector->get_input_raw();
	}
	return ret;
}

ShifterPosition ShifterTrrs::get_shifter_position(const uint32_t expire_time_ms)
{
	ShifterPosition result = ShifterPosition::SignalNotAvailable;
	if (board->is_data_valid(expire_time_ms))
	{
		uint8_t trrs = board->get_trrs();
		if (16u > trrs)
		{
			if (0u < trrs)
			{
				// check truth table
				result = TRRS_SHIFTER_TABLE[trrs];
				if (result != ShifterPosition::SignalNotAvailable) {
					this->last_valid_position = result;
				}
			}
			else
			{
				// TODO: check direction of the shift
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
	// update reverse/parking lock solenoid
	this->pos = result;
	set_rp_solenoid(vVeh, expire_time_ms);

	// return resulting shifter position
	return result;
}

void ShifterTrrs::set_rp_solenoid(const float vVeh, const uint32_t expire_time_ms)
{
	board->set_rp_solenoid(((ShifterPosition::N == this->pos) && (2.5F < vVeh)) || BrakePedal::is_brake_pedal_pressed(egs_can_hal, expire_time_ms));
}

AbstractProfile *ShifterTrrs::get_profile(const uint32_t expire_time_ms)
{
	return programselector->get_profile(expire_time_ms);
}

ShifterStyle ShifterTrrs::get_shifter_type() {
	return ShifterStyle::TRRS;
}
