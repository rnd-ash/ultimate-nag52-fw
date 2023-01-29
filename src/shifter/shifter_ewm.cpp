#include "shifter_ewm.h"

ShifterEwm::ShifterEwm(esp_err_t *can_init_status, ECU_EWM *ewm): _ewm{ewm} { }

ShifterPosition ShifterEwm::get_shifter_position(const uint64_t now, const uint64_t expire_time_ms)
{
	ShifterPosition ret = ShifterPosition::SignalNotAvailable;
	EWM_230 dest;
	if (_ewm->get_EWM_230(now, expire_time_ms, &dest))
	{
		switch (dest.get_WHC())
		{
		case EWM_230h_WHC::D:
			ret = ShifterPosition::D;
			break;
		case EWM_230h_WHC::N:
			ret = ShifterPosition::N;
			break;
		case EWM_230h_WHC::R:
			ret = ShifterPosition::R;
			break;
		case EWM_230h_WHC::P:
			ret = ShifterPosition::P;
			break;
		case EWM_230h_WHC::PLUS:
			ret = ShifterPosition::PLUS;
			break;
		case EWM_230h_WHC::MINUS:
			ret = ShifterPosition::MINUS;
			break;
		case EWM_230h_WHC::N_ZW_D:
			ret = ShifterPosition::N_D;
			break;
		case EWM_230h_WHC::R_ZW_N:
			ret = ShifterPosition::R_N;
			break;
		case EWM_230h_WHC::P_ZW_R:
			ret = ShifterPosition::P_R;
			break;
		case EWM_230h_WHC::SNV:
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
	EWM_230 ewm230;
    if (this->_ewm->get_EWM_230(now, expire_time_ms, &ewm230)) {
        result = ewm230.get_FPT();
        if (result) {
            if (!state) {
                esp_toggle = !esp_toggle;
            }
            state = true;
        }
        else {
            state = false;
        }
    }
    return result;
}
