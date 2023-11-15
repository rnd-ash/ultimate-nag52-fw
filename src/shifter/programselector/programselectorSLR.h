#ifndef PROGRAMSELECTORSLR_H
#define PRORAMSELECTORSLR_H

#include "programselector.h"
#include "../../board_config.h"

class ProgramSelectorSLR : public ProgramSelector {
public:
	explicit ProgramSelectorSLR(BoardGpioMatrix* board);
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
	ProgramSelectorType get_type() const override;
private:
	BoardGpioMatrix* board;
};

#endif