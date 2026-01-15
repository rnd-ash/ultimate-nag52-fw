
# IMPORTANT - 19/10/23 and newer FW
This firmware contains initial EGS52 calibration data. You will need to select the correct settings for your car in the configuration app under
`TCU Program settings -> CAL`
You can see [here](https://docs.ultimate-nag52.net/en/gettingstarted/configuration/calibration) for an explination on calibration settings

# 15/01/26

## Added
    * Adjustable Garage shift settings to improve garage shifting on a per-user basis
    * Adjustable parameters for the crossover shifting algorithm (Per profile type)
    * Adjustable parameters for which shifting algorithm to use and when
    * Kickdown detection (All variants)
    * Brand new TCC control algorithm
    * Calculate Pump torque for garage shifting
## Fixes
    * Utilize crossover and release algorithms under better circumstances to drastically improve comfort, even under load
    * Make the TCC locking and slipping more aggressive
    * Enable the TCC in 1st gear by default (Can be turned off by the user)
    * Fix EGS not shifting if 1 rear wheel speed sensors is defective when output shaft speed relies on wheel speed


# 16/11/25

## Added
    * Crossover shifting algorithm for much faster high powered shifting (Prevents clutch burnup)
## Fixes
    * EGS53 assembly fixes for some shifting algorithm components
    * Keep shift valves open for longer under colder temperatures (Helps prevent accidental aborted shift)
    * Crossover algorithm is now used forcefully at low RPMs (Prevents rare case of gearbox dropping gears completely)
    * TCC ramp smoothing with Zener board enabled
    * Improve torque request system
    

# 16/10/25

## Added
    * New crossover shifting algorithm for low power / coasting shifts (Helps with rough 2-1 shifting)
    * Static adder for SPC (Will be adjustable in a future release)
    * Groundwork layed for adaptations for the next release
    * Added fast pressure ramp in event of 3-4 flare to recover as quickly as possible
    * Removed unlocking of TCC when shifting
    * TCC pressure adaptation now affects higher load cells (Faster adaptation)
    * More accurate TFT sensor algorithm
    * Added gearbox load to Torque converter data diagnostics
## Fixed
    * Fixed 4matic cars with fixed ratios not being able to change gears
    * Fixed bug causing friction coefficients being wrongly calculated (Switch fallthrough)
    * Fixed PID-Correction torque varying wildly sometimes
## Other
    * MISRA-C and misc code cleanup (Thanks @chrissivo)



# 25/06/25

## Added
    * Brand new shift-release algorithm based on EGS52 reverse engineering
        * Implement PID control for RPM movement to make things more linear
        * Add in clutch protection (Forced torque request if torque goes above clutch max)
        * Manual and Race profiles are much more agressive
        * Better handle 3-2 coasting situation
        * Calculate synchronization RPM for torque request ramps
    * New pressure control system based on EGS52 reverse engineering
        * Better calculate valve body pressures
        * Clamp output pressure based on minimum working pressure
        * Calculate working pressure taking into account clutch spring pressure
    * TCC - lock more aggressive (Better fuel economy)
    * TCC - Transition pressures smoothly during gear changes
    * Embed module_settings.yml inside the firmware - It is no longer needed for TCU settings
    * Prompt upshift in manual profile and approaching redline
    * Expose the following maps as tunable
        * Low filling pressure
        * TCC slip/target pressures
    * New diagnostic data in config app (Shift algorithm information) - Displays the following:
        * Shift algorithm state for both shift and modulating sides
        * Clutch RPMs
        * Sync. RPM
        * PID algorithm control torque
        * Torque adder
        * On/Off clutch pressures
## Fixed
    * Allow TRRS shifter to be used with EGS52 CAN layer (Chrysler)
    * HFM CAN - Fix torque map not being able to store in NVS
    * Correctly handle EWM shifter without any profile selector (Sprinter/Chrysler)



# 08/12/24
* Added in shift torque data from original EGS
* Split shifting algorithm into 2 modes (Release and Crossover)
* Added support for the TCC Zener mod addon PCB (For 1.3 PCB)
* Rewrote current driver algorithm for MPC/SPC solenoids
* Added support for the Jeep 5 wire shifter (Profile switching is disabled in this mode)
    * See basic configuration for Jeep toggle setting
* Rewrote sensor API to include smoothing all sensor inputs to the TCU
* Rewrote torque API of EGS so that Each CAN layer is better handled how it sends/receives torque figures

# 13/06/24
* Improve the stability of the TCU when in emergency mode (No configuration)

# 05/06/24

## Bugs
* Adaptation is disabled at this time due to the massive rework, it will be re-enabled in a future release.

## Added
* Update ESP-IDF to 5.1.1
* Fix for keyless GO systems starting
* **Add support for EGS52 calibration data**
    * Uses OEM EGS hydraulic configuration data
    * Uses OEM EGS clutch calibration data
        * Friction coefficient table
        * Spring pressure table
    * Uses OEM EGS torque converter calibration data
        * Torque multiplication map
        * Pump torque map
* Add reverse-engineered EGS52 pressure control algorithm!
    * Computes working and solenoid inlet pressure
    * Corrects output MPC and SPC pressure accordingly
    * Computes clutch friction coeffients for optimal MPC working pressure (This means MPC working map no longer exists)
* Add in prefill for off clutch when changing gears (**NO MORE 3-4 FLARE!**)
* Clean process on first boot after update
* Add option for TCU_ADV_SETTINGS to disable torque requests for each gear change
* Sensor API rework
    * Poll all sensors on a 20ms timer tick
    * Average out VBATT and TFT signals in order to get a more steady reading
    * Speed sensors are averaged over 8 reading polls (160ms) for a much cleaner reading
* Brand new Solenoid API
    * On/Off solenoid - On for specific period of time, then activate holding phase
    * Inrush solenoid - On, Hold, Off for specific period (Torque converter solenoid)
    * CC Solenoid - Constant current driver solenoid (Corrected every 2ms)
* New torque converter (v3) algorithm
    * Adaptation maps are now present for slip and locking
    * Take into account energy through the turbine (Joules)
    * Only adapt when slip and user pedal input are stable
* New shifter input logic
    * Split the 3 style shifters into their own classes (EWM, TRRS, SLR)
    * SLR - Implement the C/M/S knob for profile selection
    * TRRS - Better handling of range restriction input
* Make output shaft sensor appear in config app if present
* DIAG `FN_SALVE_MODE` emulation (Originally on EGS52) - Allows for controlling IO of the TCU on a test bench via CAN
* DIAG `FN_CANLOGGER_MODE` - Allows for sniffing CANBUS on a vehicle (TCU does not send frames in this mode)
* EGS51 CAN Layer - Changes
    1. Read `VB` (Fuel flow) signal (Recorded in config app for parity with EGS52/53)
    2. Read `KUEB_S_A` and `KUEB_O_A` (Engine torque converter state request) bits, and pass them to torque converter control code
    3. Send `K_O_B`, `K_G_B` and `K_S_B` signals (Torque converter clutch state)
    4. Send `NEUTRAL` bit when in P/N.
    5. Send `GARAGE_SHIFT` bit when shifting from P->R or N->D (Gearbox protection under garage shifting)
* EGS52 CAN Layer - Changes
    1. Set the `HSM` (Manual mode) bit when using a profile that shifts manually (Used by ESP and Engine ECU)
    2. Send `FMRAD` (Input -> Output torque multiplier) signal (For Cruise control and ESP)
    3. Send `M_VERL` - Torque converter loss
    4. Send `K_O_B`, `K_G_B` and `K_S_B` signals (Torque converter clutch state)
    5. Read `KUEB_S_A` and `KUEB_O_A` (Engine torque converter state request) bits, and pass them to torque converter control code
* Torque requests - Add `BackToDriverDemand` torque request bit (EGS52/53). This allows the engine to know that torque will be increasing and it can roll back the ignition retarding (Results in a much smoother up ramp)
* Rework IO Expander and program selector API (Credit: @chrissivo)
* Add support for the SLR McLaren shifter!
* Add in CUSTOM_CAN CAN layer. This is to support 3rd party engine ECUs [See here for more information](https://docs.ultimate-nag52.net/en/advanced/custom-can)
* Shift logic rewrite (This is why adaptation is disabled)
    * Overlap is now split into 2 phases (Release phase, Apply and hold phase)
    * Filling of the clutches is now performed in 3 stages (High, Ramp, Low)
    * Add in engine Inertia compensation when shifting to pull the engine to the correct RPM

# 21/08/23
* EGS51 - Repair wonky torque requests
* Shift reporter - Remove warning about invalid report on flash - Since shift reporter is not being used - we can ignore it.

# 08/08/23

## Added
* EGS53 - Add support for shift paddles
* EEPROM Module settings interface
    * Allows for hot-modifying variables in the TCUs code base!
    * Wiki integration in the configuration app
* Clutch speed and velocity algorithm - based on the calculations done for the gearbox model on the wiki [here](https://docs.ultimate-nag52.net/en/resources/7226_eq)
    * Calculate the speed of the engaging and releasing clutch during the gear changes
    * Allow the TCU to dynamically set the SPC ramp based on clutch progress to target speed
    * Add RLIs so you can monitor this in the config app
* New pressure manager - Allows for programming characteristics of the valve body into the TCU so the TCU knows the max and min pressures depending on which valves are open or closed
* TRRS Range-restrict mode!
* HFM CAN (WIP)
* Remove the MPC filler adder map
* Make MPC prefill pressure += SPC prefill pressure for better prefill quality
* TCC opens up during shifting, like stock EGS to dampen the gear change for a smoother change - This will be configurable in a later update.
* Finally, smoother N-D gear engagement, however **Shifting into N-R is STILL rough**
* EGS53 - Fixed display showing 1st when in 5th gear
* Modified Input speed calculation algorithm to not use a fixed multiplier when in 1st or 5th
* Add settings to configure push button on shifter / TRRS
## Modified
* Disable TCC in 1st gear by default
* Update to PIO ESP Framework 6.3.2
* Improve gear shifting smoothness
* EGS53 - Correctly calculate driver demand torque
* EGS52 - Support TRRS shifter (For Jeep)
* Better functionality of the W/S switch (On TRRS and EWM)


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