#include "shifter_slr.h"
#include "ioexpander.h"
#include "esp_log.h"

ShifterSlr::ShifterSlr(esp_err_t *can_init_status, ECU_EWM *ewm): _ewm{ewm}
{
	*can_init_status = ioexpander->init_state();
}

ShifterPosition ShifterSlr::get_shifter_position(const uint32_t expire_time_ms)
{
	ShifterPosition ret = ShifterPosition::SignalNotAvailable;
	EWM_230_EGS52 dest;
	if (_ewm->get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &dest))
	{
		switch (dest.WHC)
		{
		case EWM_230h_WHC_EGS52::D:
			ret = ShifterPosition::D;
			break;
		case EWM_230h_WHC_EGS52::N:
			ret = ShifterPosition::N;
			break;
		case EWM_230h_WHC_EGS52::R:
			ret = ShifterPosition::R;
			break;
		case EWM_230h_WHC_EGS52::P:
			ret = ShifterPosition::P;
			break;
		case EWM_230h_WHC_EGS52::PLUS:
			ret = ShifterPosition::PLUS;
			break;
		case EWM_230h_WHC_EGS52::MINUS:
			ret = ShifterPosition::MINUS;
			break;
		case EWM_230h_WHC_EGS52::N_ZW_D:
			ret = ShifterPosition::N_D;
			break;
		case EWM_230h_WHC_EGS52::R_ZW_N:
			ret = ShifterPosition::R_N;
			break;
		case EWM_230h_WHC_EGS52::P_ZW_R:
			ret = ShifterPosition::P_R;
			break;
		case EWM_230h_WHC_EGS52::SNV:
			break;
		default:
			break;
		}
	}
	return ret;
}

AbstractProfile* ShifterSlr::get_profile(const uint32_t expire_time_ms) {

	// Profile rotate switch is connected to pins
	// 3  (PCB PRG)
	// 28 (PCB TRRS_D)
	// 27 (PCB TRRS_C)
	// 26 (PCB TRRS_B)
	// 25 (PCB TRRS_A)
	uint8_t reg[2];
	ioexpander->debug_get_registers(&reg[0], &reg[1]);
	AbstractProfile* ret = nullptr;

	switch (reg[0]) {
		case 0x24:
			ret = standard; // Right side
			break;
		case 0x26:
			ret = manual; // Center
			break;
		case 0x46:
			ret = comfort; // Left side
			break;
		default:
			break;
	}
	
	//ESP_LOGI("SLR", "DBG: %02X %02X", reg[0], reg[1]);
	return ret;
}