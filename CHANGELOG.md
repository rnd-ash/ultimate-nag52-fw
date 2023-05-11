
# Dev (Unreleased)

### Added
* EGS53 - Add support for shift paddles
#### EEPROM Module settings interface
    * TCC Settings
    * Solenoid control program settings
    * Basic shift settings
* TRRS Range-restrict mode!
* HFM CAN (WIP)
* Update to PIO ESP Framework 6.2.0
* Improve gear shifting smoothness
* Change prefill maps to account for load

# 10/04/23

### Added
* TCC learning can now reduce pressure if it detects too much biting at low load
* Input torque calculation model (Reports torque loss to ESP/ART ECU over CAN)
* Added Gearbox speed protection when garage shifting (Engine will slow down to < 1000 RPM when garage shifting
* **ADDED BACK WORKING TORQUE REQUESTS!**

#### Merged classic CAN branch	
    * Split shifter code away from CAN layers (Each shifter can now be independently selected regardless of CAN layer)
    * Add draft support for fuel and air table to calculate input torque without any torque readings on CAN (For W124 HFM)
    * Add basic petrol engine model (W124 HFM)
* Restructure a lot of the code base to be more [MISRA-C compliant](https://caxapa.ru/thumbs/468328/misra-c-2004.pdf)
### Changed
* BREAKING - Update IDF version from 4.4 to 5.0!
* Smooth TCC bite and apply algorithm for smoother applying
* Fixed bump after shifting caused by TCC re-applying
* Improved PCNT based RPM reading
* Modify the default downshift 2-1 map for C/W mode to disable 2-1 downshift at idle
* Update entire CAN Layer implementation for all EGS. Now uses bitfields instead of getter/setter functions for interacting with CAN Data
* EGS52 - Don't send shifter position when in middle positions
### Fixed
* Fixed downshift maps not being used correctly by the TCU

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