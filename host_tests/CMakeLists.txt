cmake_minimum_required(VERSION 3.16)
project(ultimate_nag52_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(host_math_tests
    test_tcu_maths.cpp
    ../lib/core/tcu_maths.cpp
)

target_include_directories(host_math_tests PRIVATE
    ../lib/core
)

add_executable(host_egs51_bugfix_tests
    test_egs51_bugfixes.cpp
    ../src/canbus/can_egs51_logic.cpp
    ../src/diag/egs_emulation_logic.cpp
)

target_include_directories(host_egs51_bugfix_tests PRIVATE
    ../src/canbus
    ../src/diag
    ../lib/egs51_ecus/src
)

add_executable(host_lowlevel_logic_tests
    test_lowlevel_logic.cpp
    ../src/ioexpander_logic.cpp
    ../src/inputcomponents/kickdownswitch_logic.cpp
    ../src/canbus/can_custom_logic.cpp
    ../src/canbus/can_egs_scaling_logic.cpp
    ../src/diag/diag_data_logic.cpp
    ../src/diag/kwp2000_logic.cpp
    ../src/diag/flasher_logic.cpp
    ../src/diag/endpoints/isotp_logic.cpp
    ../src/models/input_torque_logic.cpp
    ../src/pressure_manager_logic.cpp
    ../src/tcu_io/tcu_io_logic.cpp
    ../src/sensors_logic.cpp
)

target_include_directories(host_lowlevel_logic_tests PRIVATE
    ../src
    ../src/canbus
    ../src/diag/endpoints
    ../src/models
    ../src/tcu_io
    ../src/inputcomponents
    ../src/diag
    ../lib/customcan_ecus/src
)

add_executable(host_diag_rli33_logic_tests
    test_diag_rli33_logic.cpp
    ../src/diag/egs_emulation_rli33_logic.cpp
)

target_include_directories(host_diag_rli33_logic_tests PRIVATE
    ../src/diag
)

add_executable(host_inrush_logic_tests
    test_inrush_logic.cpp
    ../src/solenoids/inrush_solenoid_logic.cpp
)

target_include_directories(host_inrush_logic_tests PRIVATE
    ../src/solenoids
)

add_executable(host_kwp_utils_guard_tests
    test_kwp_utils_guards.cpp
    ../src/diag/kwp_utils.cpp
)

add_executable(host_kwp2000_bcd_to_hex_tests
    test_kwp2000_bcd_to_hex.cpp
    ../src/diag/kwp2000_logic.cpp
)

add_executable(host_tcu_io_data_source_tests
    test_tcu_io_data_source.cpp
    ../src/tcu_io/tcu_io_data_source.cpp
    ../src/tcu_io/tcu_io_trace_codec.cpp
)

add_executable(host_tcu_io_trace_replay
    tcu_io_trace_replay.cpp
    ../src/tcu_io/tcu_io_data_source.cpp
    ../src/tcu_io/tcu_io_trace_codec.cpp
)

# Pins the pedal scaling conversions to the literal values they replaced.
add_executable(host_pedal_scaling_tests
    test_pedal_scaling.cpp
)

target_include_directories(host_pedal_scaling_tests PRIVATE
    ../src
)

# Pins the temperature scaling conversions to the literal values they replaced.
add_executable(host_temp_scaling_tests
    test_temp_scaling.cpp
)

target_include_directories(host_temp_scaling_tests PRIVATE
    ../src
)

# Pins the torque scaling conversions, including the CAN encode saturation.
add_executable(host_torque_scaling_tests
    test_torque_scaling.cpp
)

target_include_directories(host_torque_scaling_tests PRIVATE
    ../src
)

# Pins the 2x wheel speed conversions to the bare shifts they replaced.
add_executable(host_wheel_speed_scaling_tests
    test_wheel_speed_scaling.cpp
)

target_include_directories(host_wheel_speed_scaling_tests PRIVATE
    ../src
)

# Locks the sdkconfig options that the ISR cache-safety contract depends on.
# Takes the sdkconfig path as an argument so it can run without an ESP-IDF build.
add_executable(host_sdkconfig_isr_safety_tests
    test_sdkconfig_isr_safety.cpp
)

target_include_directories(host_kwp_utils_guard_tests PRIVATE
    ../src
    ../src/diag
    ../lib/core
)

target_compile_definitions(host_kwp_utils_guard_tests PRIVATE
    KWP_UTILS_HOST_STUB=1
)

target_include_directories(host_kwp2000_bcd_to_hex_tests PRIVATE
    ../src/diag
)

target_include_directories(host_tcu_io_data_source_tests PRIVATE
    ../src
)

target_include_directories(host_tcu_io_trace_replay PRIVATE
    ../src
)

enable_testing()
add_test(NAME host_math_tests COMMAND host_math_tests)
add_test(NAME host_egs51_bugfix_tests COMMAND host_egs51_bugfix_tests)
add_test(NAME host_lowlevel_logic_tests COMMAND host_lowlevel_logic_tests)
add_test(NAME host_diag_rli33_logic_tests COMMAND host_diag_rli33_logic_tests)
add_test(NAME host_inrush_logic_tests COMMAND host_inrush_logic_tests)
add_test(NAME host_pedal_scaling_tests COMMAND host_pedal_scaling_tests)
add_test(NAME host_temp_scaling_tests COMMAND host_temp_scaling_tests)
add_test(NAME host_torque_scaling_tests COMMAND host_torque_scaling_tests)
add_test(NAME host_wheel_speed_scaling_tests COMMAND host_wheel_speed_scaling_tests)
add_test(
    NAME host_sdkconfig_isr_safety_tests
    COMMAND host_sdkconfig_isr_safety_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../sdkconfig.unified
)
add_test(NAME host_kwp_utils_guard_tests COMMAND host_kwp_utils_guard_tests)
add_test(NAME host_kwp2000_bcd_to_hex_tests COMMAND host_kwp2000_bcd_to_hex_tests)
add_test(NAME host_tcu_io_data_source_tests COMMAND host_tcu_io_data_source_tests)
add_test(
    NAME host_tcu_io_trace_replay_tests
    COMMAND host_tcu_io_trace_replay
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_input_sample.csv
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_expected_actuator.csv
)
add_test(
    NAME host_tcu_io_trace_replay_tolerant_tests
    COMMAND host_tcu_io_trace_replay
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_input_sample.csv
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_expected_actuator_tolerant.csv
    --tolerance-ma 5
    --tolerance-pwm 10
    --quiet
)
add_test(
    NAME host_tcu_io_trace_replay_config_tests
    COMMAND host_tcu_io_trace_replay
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_input_sample.csv
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_expected_actuator_tolerant.csv
    --tolerance-config
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_tolerance_profile.cfg
    --quiet
)
add_test(
    NAME host_tcu_io_trace_replay_wildcard_tests
    COMMAND host_tcu_io_trace_replay
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_input_sample.csv
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_expected_actuator_wildcard.csv
    --quiet
)
add_test(
    NAME host_tcu_io_trace_replay_metrics_tests
    COMMAND host_tcu_io_trace_replay
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_input_sample.csv
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_expected_actuator.csv
    ${CMAKE_CURRENT_BINARY_DIR}/tcu_io_trace_actual_out.csv
    --metrics-out
    ${CMAKE_CURRENT_BINARY_DIR}/tcu_io_trace_metrics.txt
    --quiet
)
add_test(
    NAME host_tcu_io_trace_replay_scenario_tests
    COMMAND host_tcu_io_trace_replay
    --scenario
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tcu_io_trace_scenario.cfg
    --metrics-out
    ${CMAKE_CURRENT_BINARY_DIR}/tcu_io_trace_scenario_metrics.txt
    --quiet
)
