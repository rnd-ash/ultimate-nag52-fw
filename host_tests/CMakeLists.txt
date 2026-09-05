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

enable_testing()
add_test(NAME host_math_tests COMMAND host_math_tests)
