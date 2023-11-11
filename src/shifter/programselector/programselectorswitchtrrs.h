#ifndef PROGRAMSELECTORSWITCHTRRS_H
#define PROGRAMSELECTORSWITCHTRRS_H

#include "programselectorswitch.h"
#include "../../board_config.h"

class ProgramSelectorSwitchTRRS : public ProgramSelectorSwitch{
public:
	ProgramSelectorSwitchTRRS(BoardGpioMatrix* board, AbstractProfile* profiles);
	AbstractProfile* get_profile(const uint32_t expire_time_ms) override;
private:
	BoardGpioMatrix* board;
};

#endif