#ifndef SHIFTER_SLR_H
#define SHIFTER_SLR_H

#include "shifter.h"
#include "esp_err.h"
#include "../../egs52_ecus/src/EWM.h"

class ShifterSlr : public Shifter
{
public:
    explicit ShifterSlr(esp_err_t *can_init_status, ECU_EWM *ewm);
    ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
	bool get_profile_btn_press(const uint32_t expire_time_ms);
    AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
private:
    ECU_EWM *_ewm;
};

#endif // SHIFTER_TRRS_H