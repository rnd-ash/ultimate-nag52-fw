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

Test binary target: `host_math_tests`
