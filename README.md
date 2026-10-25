# Firmware for Ultimate NAG52 722.6 TCU

The Ultimate-NAG52 project is a full CAN supported custom TCU for the 722.6 gearbox, built around the ESP32 controller,
supporting *most* vehicles that came with the EGS51, EGS52 or EGS53 TCUs!

There is also a 'custom-can' layer for 3rd party engine ECUs (See documentation on the wiki for the CAN Layer message spec).

## Vehicle exceptions (Cars that cannot run this TCU)
* Cars that come with the SBW (Shift-by-wire) steering wheel gear selector
* Porsche vehicles

## Features
* Full CAN support for vehicles that came with EGS51/EGS52 or EGS53 TCUs.
* Supports both gear selector types (TRRS `4321` and Tiptronic `+/-` shifter)
* CAN Support for M104 HFM ECU (For W124 swaps)
* Fully-configurable maps and calibrations (Using the [Ultimate-NAG52 config app](https://github.com/rnd-ash/ultimate-nag52-config-app))
* Adaptations
  * Torque converter adapts pressure for each gear based on load and temperature
  * Gear shifting adaptation - Clutch filling pressure, Clutch filling time and correction torque
* 5 profiles (Each with configurable shift point and shift speed maps)
  * (S)tandard - Standard base map
  * (C)omfort - Smoother shifting, starts in 2nd gear
  * (W)inter - Comfortable shift speed, manual shifting, starts in 2nd gear
  * (M)anual - Full manual mode (Paddle/Tiptronic cars only)
  * (R)ace - Super fast and aggressive shifts that still feel smooth - Manual shifting
* Optional output shaft speed sensor support (TCU Pin 23), to bypass the requirement of ABS in some custom builds
* Configurable vehicle parameters (Redline, Differential ratio, Wheel size, Gearbox type)
* Diagnostics over USB or CAN (With the configuration app)

## Links
* [YT playlist](https://www.youtube.com/watch?v=V27D5PrgMk8&list=PLxrw-4Vt7xtu9d8lCkMCG0_K7oHcsSMtF)
* [Store website](http://ultimate-nag52.net/)
* [Wiki](http://docs.ultimate-nag52.net/)

## DISCLAIMER

Although running this TCU in your vehicle is mostly safe, transmission failure can occur once you start modifying maps and calibrations.
I take no responsibility for failure of gearboxes due to firmware or map modifications.


## NOTICE ABOUT COPIED CODE (For PRs or change requests)
This project contains code built from the ground up, or code based on black-box style reverse engineering of the behaviour of the standard EGS52 module. PRs based on unethically sourced documentation or code will be immediately deleted and closed. 

## AI contributions

Contributions that are AI-assisted are welcome, however they MUST be reviewed by a human first. If the PR is written entirely by AI, it will be closed.

(If you do use AI to assist you in writing PRs, you must include in the commit message that it was AI assisted)

## Additional contributors 

* @chrissivo - ClassicCAN contributor (For M104 HFM cars)
* @trampas - Code hardening and bug fixing
