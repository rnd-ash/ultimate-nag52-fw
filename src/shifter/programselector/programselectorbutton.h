#ifndef PROGRAMSELECTORBUTTON_H
#define PRORAMSELECTORBUTTON_H

#include "programselector.h"
#include "../../nvs/eeprom_config.h"
#include "../../canbus/can_hal.h"

class ProgramSelectorButton : public ProgramSelector
{
private:
	TCM_CORE_CONFIG* vehicle_config;
	bool is_pressed_last_call = false;
public:
	ProgramSelectorButton(TCM_CORE_CONFIG* vehicle_config, AbstractProfile** profiles);
	void set_button_pressed(const bool is_pressed);
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
};

#endif // PRORAMSELECTORBUTTON_H