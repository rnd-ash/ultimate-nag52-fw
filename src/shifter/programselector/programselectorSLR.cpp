#include "programselectorSLR.h"
#include "../../clock.hpp"
#include "../../nvs/module_settings.h"

ProgramSelectorSLR::ProgramSelectorSLR(BoardGpioMatrix* board) : board(board)
{
}

AbstractProfile *ProgramSelectorSLR::get_profile(const uint32_t expire_time_ms)
{
	AbstractProfile *result = nullptr;
	// Profile rotate switch is connected to pins
	// 3  (PCB PRG)
	// 28 (PCB TRRS_D)
	// 27 (PCB TRRS_C)
	// 26 (PCB TRRS_B)
	// 25 (PCB TRRS_A)
	uint8_t reg[2];
	ioexpander->debug_get_registers(&reg[0], &reg[1]);
	switch (reg[0]) {
		case 0x24:
			result = comfort; // Right side
			break;
		case 0x26:
			result = manual; // Center
			break;
		case 0x46:
			result = standard; // Left side
			break;
		default:
			break;
	}
	return result;
}

ProgramSelectorType ProgramSelectorSLR::get_type() const {
	return ProgramSelectorType::SLR;
}
