#ifndef PROGRAMSELECTORSWITCH_H
#define PROGRAMSELECTORSWITCH_H

#include "programselector.h"

class ProgramSelectorSwitch : public ProgramSelector
{
public:
protected:
    /**
     * @brief used for TRRS profile switch and EWM profile switch position
     */
    enum class ProfileSwitchPos : uint8_t
    {
        /**
         * @brief Top position (S)
         */
        Top = 0u,
        /**
         * @brief Bottom position (C or W)
         */
        Bottom = 1u,
        /**
         * @brief Could not determine position
         */
        SNV = UINT8_MAX,
    };
};
#endif // PROGRAMSELECTORSWITCH_H