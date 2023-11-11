#ifndef PROGRAMSELECTORSWITCHEWM_H
#define PROGRAMSELECTORSWITCHEWM_H

#include "programselectorswitch.h"
#include "programselectorewm.h"

class ProgramSelectorSwitchEWM : public ProgramSelectorSwitch, ProgramSelectorEWM {
public:
	explicit ProgramSelectorSwitchEWM(AbstractProfile* profiles);
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
};
#endif // PROGRAMSELECTORSWITCHEWM_H