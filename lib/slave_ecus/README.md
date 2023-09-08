
# What is this?

CAN definitions for EGS slave mode (Accessed in any EGS CAN mode)

## What is slave mode?

Slave mode is accessed via `FN_SLAVE_MODE` in Vediamo (After Siemens level security access `0xFE` level), and allows for controlling the TCU outputs directly over CANBUS. In this mode, the TCU also sends data over CAN about its recorded inputs.

When in this mode, the TCU is completely non functional in a vehicle. No data is sent over CAN for normal operation. This mode is SPECIFICALLY designed (Likely) for verifying the TCU functions when on a production line.

## Why does Ultimate-NAG52 need this?

1. If the OEM TCU has this feature - and UN52 is a faithful replacement of the OEM EGS, then this TCU also needs to have it.
2. It is useful for verifying the TCU functionality easily without having to insert the TCU into a car for it to start actuating solenoids


### A kind notice to anyone at Siemens or Mercedes who might read this
Siemens level security access was very easy to gain once I have access to my cars original EGS52 flash file (It was read on an external flash reader). For better security, I'd suggest NOT burning your master seed and key directly into the flash! The CAN defintiions for this were super easy to reverse engineer due to only a few parameters being present.

## Note
CAN ID 0x50C was added to allow for reporting of extra outputs that the Ultimate-NAG52 TCU does. The original EGS modules do not transmit this CAN ID.

