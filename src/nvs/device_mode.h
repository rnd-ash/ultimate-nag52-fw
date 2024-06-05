#ifndef __DEVICE_MODE_H_
#define __DEVICE_MODE_H_

#include <stdint.h>

// Wrapped into uint16_t
extern uint16_t CURRENT_DEVICE_MODE;

#define CHECK_MODE_BIT_ENABLED(bit) ((CURRENT_DEVICE_MODE & bit) == bit)


#define DEVICE_MODE_NORMAL BIT(0) // Normal operation
// BIT 1?
#define DEVICE_MODE_ROLLER BIT(2) // Self test mode (In CBF)
#define DEVICE_MODE_SLAVE BIT(3) // In CBF - Diagnostic CAN data available and solenoid control
#define DEVICE_MODE_TEMPORARY_ERROR BIT(4) // Limp mode (Disabled after ignition cycle)
// BIT 5?
#define DEVICE_MODE_ERROR BIT(6) // Limp mode (Until DTCs cleared)
#define DEVICE_MODE_NO_CALIBRATION BIT(7) // No calibration (Cannot init)
// BIT 8?
// BIT 9?
// BIT 10?
// BIT 11?
// BIT 12?
// BIT 13?
// BIT 14?
#define DEVICE_MODE_CANLOGGER BIT(15) // UN52 specific (CAN logging available)

#endif