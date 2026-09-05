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
    ../src/sensors_logic.cpp
)

target_include_directories(host_lowlevel_logic_tests PRIVATE
    ../src
    ../src/canbus
    ../src/inputcomponents
    ../lib/customcan_ecus/src
)

add_executable(host_diag_rli33_logic_tests
    test_diag_rli33_logic.cpp
    ../src/diag/egs_emulation_rli33_logic.cpp
)

target_include_directories(host_diag_rli33_logic_tests PRIVATE
    ../src/diag
)

enable_testing()
add_test(NAME host_math_tests COMMAND host_math_tests)
add_test(NAME host_egs51_bugfix_tests COMMAND host_egs51_bugfix_tests)
add_test(NAME host_lowlevel_logic_tests COMMAND host_lowlevel_logic_tests)
add_test(NAME host_diag_rli33_logic_tests COMMAND host_diag_rli33_logic_tests)
