# 19/02/23

## Added
* SHIFT - Realtime SPC and MPC modulation during shifts based on input shaft torque (SUPER SMOOTH SHIFTS)
* SHIFT - Cooldown after same solenoid activation to avoid failed shifts
* TCC  - locking learning adaptation (Manually adjustable in config app too)
* SHIFT - MPC cushion map for increasing MPC pressure in fill phase (Fix for 3-4 flare)
* SENSORS - Added Output shaft PCNT code for Output shaft RPM sensor via GPIO
* SENSORS - Input shaft sensor debouncing for more accurate input RPM readings
* EGS52 (Chrysler) - Add support for W/S button on EWM shifters
* GEARBOX - Added AC compressor torque loss when calcualting input shaft torque

## Changed
* DIAG  - Updated FW upload/download API - More inline with KWP2000 (New 'updater' page in config app required)
* EGS5x - Optimized CAN message Rx
* EGS52 - Correct FMRAD wheel torque parity (Prevents P20D6 'EGS Momentenanforderung Parityfehler' on ME SFI 2.8 ECU)
* EGS52 - Removed torque requests - Too aggressive on CDI ECUs
* SHIFT - Increase Shift solenoid current inrush time to 1 second for sticky old solenoids
* EGS51 - Fix Start Enable pin on with board V1.3
* EGS51 - Update torque getter functions

## Known bugs
* 2-1 shift map values don't work at low input RPM

# Initial release - 03/01/23