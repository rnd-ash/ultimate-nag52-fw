#ifndef PROGRAMSELECTORSWITCHEWM_H
#define PROGRAMSELECTORSWITCHEWM_H

#include "programselector.hpp"

class ProgramSelectorSwitchEWM : public ProgramSelector {
public:
	explicit ProgramSelectorSwitchEWM();
	void set_profile_switch_pos(ProfileSwitchPos pos);
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
	ProgramSelectorType get_type() const override;
	DiagProfileInputState get_input_raw() const override;
private:
	ProfileSwitchPos pos = ProfileSwitchPos::SNV;
};
#endif // PROGRAMSELECTORSWITCHEWM_H