#ifndef __LEGACY_SHIFTER_H__
#define __LEGACY_SHIFTER_H__

#include "gearbox_config.h"
#include "canbus/can_hal.h"
#ifdef BOARD_V2

// PIN IO on the IO expander
typedef struct {
    uint8_t pin_id;
    uint8_t port;
} GPIO_PIN;

const static GPIO_PIN GPIO_TRRS_A_SENSE   = { 0, 0 }; // TRRS A (Input)
const static GPIO_PIN GPIO_TRRS_B_SENSE   = { 1, 0 }; // TRRS B (Input)
const static GPIO_PIN GPIO_TRRS_C_SENSE   = { 2, 0 }; // TRRS C (Input)
const static GPIO_PIN GPIO_TRRS_D_SENSE   = { 3, 0 }; // TRRS D (Input)
const static GPIO_PIN GPIO_PROG_BTN_SENSE = { 4, 0 }; // Program selection switch (Input)
const static GPIO_PIN GPIO_BRAKE_SENSE    = { 5, 0 }; // Brake switch (Input)
const static GPIO_PIN GPIO_KD_SENSE       = { 6, 0 }; // Kickdown switch (Input)

const static GPIO_PIN GPIO_RP_SOL_EN = { 1, 1 }; // Reverse/Park lockout solenoid (Output)
const static GPIO_PIN GPIO_START_EN  = { 0, 1 }; // Start enable contact solenoid (Output)

const static uint8_t GPIO_I2C_ADDR = 0x20; 


enum class RegisterAddr : uint8_t {
    P0_INPUT    = 0x00,
    P0_OUTPUT   = 0x02,
    P0_POLINV   = 0x04,
    P0_CONFIG   = 0x06,
    P0_DRVSTR1  = 0x40,
    P0_DRVSTR2  = 0x41,
    P0_ILATCH   = 0x44,
    P0_PULLENA  = 0x46,
    P0_PULLSEL  = 0x48,
    P0_INTMASK  = 0x4A,
    P0_INTSTAT  = 0x4C,
    P1_INPUT    = 0x01,
    P1_OUTPUT   = 0x03,
    P1_POLINV   = 0x05,
    P1_CONFIG   = 0x07,
    P1_DRVSTR1  = 0x42,
    P1_DRVSTR2  = 0x43,
    P1_ILATCH   = 0x45,
    P1_PULLENA  = 0x47,
    P1_PULLSEL  = 0x49,
    P1_INTMASK  = 0x4B,
    P1_INTSTAT  = 0x4D,
    OUTPUT_CONF = 0x4F
};

// Supporting legacy non-CAN shifter mechanism via the I2C IO Expander (PCA9535PW)
// Also supports some functions that get replaced in EGS52_CAN.h 
class LegacyShifter {
public:
    LegacyShifter();
    ShifterPosition get_shifter_position();
    /**
     * Unlike EWM module, this shifter does NOT
     * have a button for switching profiles.
     * Instead, it has a toggle switch between
     * W and S mode
     */
    bool prog_button_position();
    bool brake_pressed();
    bool kickdown_pressed();

    void set_rp_solenoid(bool state);
    void set_start_enabled(bool enable);
};


#endif // BOARD_V2
#endif // LEGACY_SHIFTER