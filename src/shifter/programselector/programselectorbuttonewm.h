#ifndef PROGRAMSELECTORBUTTONEWM_H
#define PRORAMSELECTORBUTTONEWM_H

#include "programselector.hpp"
#include "../../nvs/eeprom_config.h"
#include "../../canbus/can_hal.h"



class ProgramSelectorButtonEwm : public ProgramSelector
{
private:
	TCM_CORE_CONFIG* vehicle_config;
	bool is_pressed_last_call = false;
public:
	explicit ProgramSelectorButtonEwm(TCM_CORE_CONFIG* vehicle_config);
	void set_button_pressed(const bool is_pressed);
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
	ProgramSelectorType get_type() const override;
	DiagProfileInputState get_input_raw() const override;
};

#endif // PRORAMSELECTORBUTTON_H