#ifndef PROGRAMSELECTORSWITCHTRRS_H
#define PROGRAMSELECTORSWITCHTRRS_H

#include "programselector.hpp"
#include "../../board_config.h"

class ProgramSelectorSwitchTRRS : public ProgramSelector {
public:
	explicit ProgramSelectorSwitchTRRS(BoardGpioMatrix* board);
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
	ProgramSelectorType get_type() const override;
	DiagProfileInputState get_input_raw() const override;
private:
	BoardGpioMatrix* board;
};

#endif