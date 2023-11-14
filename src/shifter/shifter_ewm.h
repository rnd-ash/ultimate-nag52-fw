#ifndef SHIFTER_EWM_H
#define SHIFTER_EWM_H

#include "shifter.h"
#include "esp_err.h"
#include "../nvs/module_settings.h"

class ShifterEwm : public Shifter
{
public:
	ShifterEwm(TCM_CORE_CONFIG* vehicle_config, ETS_MODULE_SETTINGS* shifter_settings);
	ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
	void set_program_button_pressed(const bool is_pressed);
private:
	bool state = false;
	bool esp_toggle = false;
	ProgramSelector* programselector;
};

#endif // SHIFTER_EWM_H