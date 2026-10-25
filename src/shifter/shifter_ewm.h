#ifndef SHIFTER_EWM_H
#define SHIFTER_EWM_H

#include "shifter.h"
#include "esp_err.h"
#include "../nvs/module_settings.h"
#include "programselector/programselector.hpp"
#include "../../egs52_ecus/src/EWM.h"

class ShifterEwm : public Shifter
{
public:
	ShifterEwm(const ETS_MODULE_SETTINGS* shifter_settings);
	ShifterPosition get_shifter_position(void) override;
	AbstractProfile* get_profile(void) override;
	void set_program_button_pressed(const bool is_pressed, const ProfileSwitchPos pos);
	DiagProfileInputState diag_get_profile_input(void) override;
	ShifterStyle get_shifter_type(void) override;
	void update(void) override;
private:
	const uint32_t expire_time_ms = 250u;
	bool state = false;
	bool esp_toggle = false;
	ProgramSelector* programselector;
};

#endif // SHIFTER_EWM_H