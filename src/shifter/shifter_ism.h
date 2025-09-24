#ifndef SHIFTER_ISM_H
#define SHIFTER_ISM_H

#include "shifter.h"
#include "esp_err.h"
#include "../nvs/module_settings.h"
#include "programselector/programselector.hpp"
#include "../../egs53_ecus/src/ANY_ECU.h"
#include "../canbus/can_egs53.h"

class ShifterIsm : public Shifter
{
public:
	ShifterIsm(TCM_CORE_CONFIG* vehicle_config, ETS_MODULE_SETTINGS* shifter_settings);
	// For special handling
	void update(Egs53Can* can);
	ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
	void set_program_button_pressed(const bool is_pressed, const ProfileSwitchPos pos);
	DiagProfileInputState diag_get_profile_input() override;
	ShifterStyle get_shifter_type() override;
private:
	bool state = false;
	bool esp_toggle = false;
	ProgramSelector* programselector;
};

#endif // SHIFTER_EWM_H