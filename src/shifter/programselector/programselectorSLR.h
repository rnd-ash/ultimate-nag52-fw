#ifndef PROGRAMSELECTORSLR_H
#define PRORAMSELECTORSLR_H

#include "programselector.hpp"
#include "../../board_config.h"

/**
 * @brief SLR profile knob position
 */
// enum class SLRProfileWheel : uint8_t {
    /**
     * @brief Right (S)
     */
    // Right = 0u,
    /**
     * @brief Center (M)
     */
//     Center = 1u,
    /**
     * @brief Left (C)
     */
//     Left = 2u,
    /**
     * @brief Could not determine position
     */
//     SNV = UINT8_MAX,
// };


class ProgramSelectorSLR : public ProgramSelector {
public:
	explicit ProgramSelectorSLR(BoardGpioMatrix* board);
	AbstractProfile* get_profile(void) override;
	ProgramSelectorType get_type(void) const override;
	DiagProfileInputState get_input_raw(void) const override;
private:
	BoardGpioMatrix* board;
};

#endif