#ifndef SHIFTER_EWM_H
#define SHIFTER_EWM_H

#include "shifter.h"
#include "esp_err.h"
#include "../../egs52_ecus/src/EWM.h"

class ShifterEwm : public Shifter
{
public:
	ShifterEwm(esp_err_t *can_init_status, ECU_EWM *ewm);
	ShifterPosition get_shifter_position(const uint64_t now, const uint64_t expire_time_ms) override;
	bool get_profile_btn_press(uint64_t now, uint64_t expire_time_ms);
private:
	ECU_EWM *_ewm;
	bool state = false;
	bool esp_toggle = false;
};

#endif // SHIFTER_EWM_H