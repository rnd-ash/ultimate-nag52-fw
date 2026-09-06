# Host Tests (x86, No Board Required)

This folder contains native unit tests that run on your local machine via WSL (Ubuntu), without requiring ESP32 hardware.

## Prerequisites
- WSL distro with `g++`, `cmake`, and `ctest`

## Run
From Windows PowerShell at repo root:

```powershell
wsl -d Ubuntu-22.04 bash -lc "set -e; cd /mnt/d/Projects/E300/ultimate-nag52-fw/host_tests; cmake -S . -B build; cmake --build build -j; ctest --test-dir build --output-on-failure"
```

## What Is Covered
- `interpolate_float` clamping and inverted-axis behavior
- `interpolate_int` clamping
- `first_order_filter` integer and float behavior (including `0xFF` guard path)
- `linear_ramp_with_timer`
- `linear_interp_with_percentage`

## Hardware Playback Scaffolding
- `TCUIO` now supports pluggable frame providers for simulation.
- New abstraction types live in [src/tcu_io/tcu_io_data_source.h](../src/tcu_io/tcu_io_data_source.h).
- `PlaybackTcuIoDataSource` replays captured samples on x86.
- `TcuIoFrameRingBufferCaptureSink` can capture live frames into a fixed-size ring buffer.
- `ITcuIoActuatorController` routes commanded solenoid outputs to either real hardware (`live`) or test doubles (`mock`).
- `TcuIoActuatorRingBufferCaptureSink` captures actuator command history for assertions and regression replay.
- Host target `host_tcu_io_data_source_tests` validates playback looping and capture behavior.

## Replay Tool
- `host_tcu_io_trace_replay` replays a hardware trace CSV into a simulation policy and emits actuator commands.
- CSV parsing/formatting helpers are in [src/tcu_io/tcu_io_trace_codec.h](../src/tcu_io/tcu_io_trace_codec.h).
- Sample traces are in [host_tests/data](data).
- One-command scenarios are supported with `--scenario <manifest.cfg>`.
- Comparison supports tolerances for numeric actuator fields using `--tolerance`, `--tolerance-ma`, and `--tolerance-pwm`.
- Per-scenario tolerance profiles are supported via `--tolerance-config <path>` (see `host_tests/data/tcu_io_trace_tolerance_profile.cfg`).
- Expected actuator CSV supports per-field wildcards (`*`) to ignore specific values on a given frame.
- Optional metrics output is available via `--metrics-out <path>`.

Example:

```powershell
wsl -d Ubuntu-22.04 bash -lc "set -e; cd /mnt/d/Projects/E300/ultimate-nag52-fw/host_tests/build_wsl; ./host_tcu_io_trace_replay ../data/tcu_io_trace_input_sample.csv ../data/tcu_io_trace_expected_actuator.csv"
```

Tolerance example:

```powershell
wsl -d Ubuntu-22.04 bash -lc "set -e; cd /mnt/d/Projects/E300/ultimate-nag52-fw/host_tests/build_wsl; ./host_tcu_io_trace_replay ../data/tcu_io_trace_input_sample.csv ../data/tcu_io_trace_expected_actuator_tolerant.csv --tolerance-ma 5 --tolerance-pwm 10 --quiet"
```

Config-profile example:

```powershell
wsl -d Ubuntu-22.04 bash -lc "set -e; cd /mnt/d/Projects/E300/ultimate-nag52-fw/host_tests/build_wsl; ./host_tcu_io_trace_replay ../data/tcu_io_trace_input_sample.csv ../data/tcu_io_trace_expected_actuator_tolerant.csv --tolerance-config ../data/tcu_io_trace_tolerance_profile.cfg --quiet"
```

Wildcard example:

```powershell
wsl -d Ubuntu-22.04 bash -lc "set -e; cd /mnt/d/Projects/E300/ultimate-nag52-fw/host_tests/build_wsl; ./host_tcu_io_trace_replay ../data/tcu_io_trace_input_sample.csv ../data/tcu_io_trace_expected_actuator_wildcard.csv --quiet"
```

Scenario manifest example:

```powershell
wsl -d Ubuntu-22.04 bash -lc "set -e; cd /mnt/d/Projects/E300/ultimate-nag52-fw/host_tests/build_wsl; ./host_tcu_io_trace_replay --scenario ../data/tcu_io_trace_scenario.cfg --metrics-out ./tcu_io_trace_scenario_metrics.txt --quiet"
```

Scenario manifest keys (`key=value`):
- `input_csv`
- `expected_csv`
- `output_csv`
- `metrics_out`
- `tolerance_config`
- `tolerance`, `tolerance_ma`, `tolerance_pwm`, `tolerance_mpc_ma`, `tolerance_spc_ma`, `tolerance_tcc_pwm`
- `check_y3`, `check_y4`, `check_y5` (0/1)
- `wildcard_policy` (`allow`/`forbid`) or `allow_expected_wildcards` (0/1)
- `quiet` (0/1)

Paths in manifests are resolved relative to the manifest file path.

Test binary target: `host_math_tests`
