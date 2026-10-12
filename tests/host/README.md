# Firmware fault-handling regressions

Run from the repository root:

```sh
python3 tests/host/run_fault_fixes.py
```

Requires Python 3 and g++. Each program extracts current production methods or
arithmetic and compiles them with explicit fake CAN, solenoid, map or flash
interfaces. The shared runner enables AddressSanitizer and UndefinedBehaviorSanitizer;
only packed-struct alignment checks are excluded because host layout differs
from the embedded ABI. Leak detection remains enabled.

Coverage: missing/recovered/stopped engine RPM, torque suppression, retained gear
state and guarded pressure shutdown; engine Open and all four configured TCC
shift unlocks from filling and steady slip; disabled-gear release and normal
hysteresis; incremental SPC adaptation and scaled bounds; calibration replacement
validation, repeated failure cleanup and valid replacement of invalid old RAM.

These are focused host regressions, not a full controller simulation, physical
hydraulic model, or vehicle validation. They never upload firmware or access a TCM.

Pedal qualification regressions: `python3 -m unittest discover -s tests/host -v`.
These additionally cover the complete raw byte domain, pending road-gear requests,
active-shift ownership, range changes, torque suppression and recovery.
