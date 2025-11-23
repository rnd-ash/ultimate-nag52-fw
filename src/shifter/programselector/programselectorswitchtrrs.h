#ifndef PROGRAMSELECTORSWITCHTRRS_H
#define PROGRAMSELECTORSWITCHTRRS_H

#include "programselector.hpp"
#include "../../board_config.h"

class ProgramSelectorSwitchTRRS : public ProgramSelector {
public:
	explicit ProgramSelectorSwitchTRRS(BoardGpioMatrix* board);
	AbstractProfile* get_profile(void) override;
	ProgramSelectorType get_type(void) const override;
	DiagProfileInputState get_input_raw(void) const override;
private:
	BoardGpioMatrix* board;
	const uint32_t expire_time_IC_query = 500u;
};

#endif