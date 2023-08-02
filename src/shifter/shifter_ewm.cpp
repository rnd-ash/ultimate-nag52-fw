#include "shifter_ewm.h"

ShifterEwm::ShifterEwm(esp_err_t *can_init_status, ECU_EWM *ewm): _ewm{ewm} { }

ShifterPosition ShifterEwm::get_shifter_position(const uint64_t now, const uint64_t expire_time_ms)
{
	ShifterPosition ret = ShifterPosition::SignalNotAvailable;
	EWM_230_EGS52 dest;
	if (_ewm->get_EWM_230(now, expire_time_ms, &dest))
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

bool ShifterEwm::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms)
{
	bool result = false;
	EWM_230_EGS52 ewm;
    if (this->_ewm->get_EWM_230(now, expire_time_ms, &ewm)) {
        result = ewm.FPT;
    }
    return result;
}

ProfileSwitchPos ShifterEwm::get_shifter_profile_switch_pos(const uint64_t now, const uint64_t expire_time_ms) {
	ProfileSwitchPos result = ProfileSwitchPos::SNV;
	EWM_230_EGS52 ewm;
    if (this->_ewm->get_EWM_230(now, expire_time_ms, &ewm)) {
		result = ewm.W_S ? ProfileSwitchPos::Top : ProfileSwitchPos::Bottom;
	}
	return result;
}
