#include "shifter_ism.h"
#include "programselector/programselectorbuttonewm.h"
#include "programselector/programselectorswitchewm.h"

ShifterIsm::ShifterIsm(ETS_MODULE_SETTINGS *shifter_settings)
{
	programselector = nullptr;
	ESP_LOGW("ISM", "ISM Shifter selected. This is experimental!");
}

DiagProfileInputState ShifterIsm::diag_get_profile_input() {
	return DiagProfileInputState::None;
}

ShifterPosition ShifterIsm::get_shifter_position(const uint32_t expire_time_ms)
{
	return ShifterPosition::SignalNotAvailable;
}

AbstractProfile *ShifterIsm::get_profile(const uint32_t expire_time_ms)
{
	return nullptr;
}

void ShifterIsm::set_program_button_pressed(const bool is_pressed, const ProfileSwitchPos pos)
{
}

ShifterStyle ShifterIsm::get_shifter_type() {
	return ShifterStyle::ISM;
}

void ShifterIsm::update(Egs53Can* can) {
	//ESP_LOGI("ISM", "Update!");
}