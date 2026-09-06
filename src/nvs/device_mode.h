#ifndef __DEVICE_MODE_H_
#define __DEVICE_MODE_H_

#include <stdint.h>

// Wrapped into uint16_t
extern uint16_t CURRENT_DEVICE_MODE;

#define CHECK_MODE_BIT_ENABLED(bit) ((((uint16_t)(CURRENT_DEVICE_MODE)) & ((uint16_t)(bit))) == ((uint16_t)(bit)))


#define DEVICE_MODE_NORMAL (BIT(0)) // Normal operation
// BIT 1?
#define DEVICE_MODE_ROLLER (BIT(2)) // Self test mode (In CBF)
#define DEVICE_MODE_SLAVE (BIT(3)) // In CBF - Diagnostic CAN data available and solenoid control
#define DEVICE_MODE_TEMPORARY_ERROR (BIT(4)) // Limp mode (Disabled after ignition cycle)
// BIT 5?
#define DEVICE_MODE_ERROR (BIT(6)) // Limp mode (Until DTCs cleared)
#define DEVICE_MODE_NO_CALIBRATION (BIT(7)) // No calibration (Cannot init)
#define DEVICE_MODE_NO_EFUSE (BIT(8)) // No Efuse set (Cannot init)
// BIT 9?
// BIT 10?
// BIT 11?
// BIT 12?
// BIT 13?
// BIT 14?
#define DEVICE_MODE_CANLOGGER (BIT(15)) // UN52 specific (CAN logging available)

// Every bit that has a defined meaning. Used to reject device mode writes that
// arrive over the diagnostic link with unknown bits set.
#define DEVICE_MODE_VALID_MASK ((uint16_t)( \
    DEVICE_MODE_NORMAL | \
    DEVICE_MODE_ROLLER | \
    DEVICE_MODE_SLAVE | \
    DEVICE_MODE_TEMPORARY_ERROR | \
    DEVICE_MODE_ERROR | \
    DEVICE_MODE_NO_CALIBRATION | \
    DEVICE_MODE_NO_EFUSE | \
    DEVICE_MODE_CANLOGGER))

#endif