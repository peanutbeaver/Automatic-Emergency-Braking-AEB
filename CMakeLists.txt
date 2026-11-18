cmake_minimum_required(VERSION 3.25)
project(automatic_braking VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(AEB_WERROR "Treat warnings as errors in production code" ON)
option(AEB_SANITIZE "Build host code with AddressSanitizer + UBSan" OFF)

set(AEB_FIRMWARE_BUILD OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Generic")
  set(AEB_FIRMWARE_BUILD ON)
endif()

# Strict warnings for the production code (controller + plant model).
set(AEB_STRICT_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
                        -Wdouble-promotion -Wcast-align -Wnull-dereference)
if(AEB_WERROR)
  list(APPEND AEB_STRICT_WARNINGS -Werror)
endif()

if(NOT AEB_FIRMWARE_BUILD)
  # Forbid fused multiply-add so host floats match the Cortex-M3 soft-float
  # results bit for bit (REQ-AEB-091).
  add_compile_options(-ffp-contract=off)
  if(AEB_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

add_library(aeb_core STATIC src/aeb_controller.cpp)
target_include_directories(aeb_core PUBLIC include)
target_compile_options(aeb_core PRIVATE ${AEB_STRICT_WARNINGS})

add_library(aeb_sim STATIC sim/src/scenarios.cpp sim/src/simulation.cpp sim/src/text_buffer.cpp)
target_include_directories(aeb_sim PUBLIC sim/include)
target_link_libraries(aeb_sim PUBLIC aeb_core)
target_compile_options(aeb_sim PRIVATE ${AEB_STRICT_WARNINGS})

if(AEB_FIRMWARE_BUILD)
  add_subdirectory(target)
  return()
endif()

add_executable(aeb_sil sim/host/main.cpp)
target_link_libraries(aeb_sil PRIVATE aeb_sim)
target_compile_options(aeb_sil PRIVATE ${AEB_STRICT_WARNINGS})

enable_testing()
find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
  FetchContent_MakeAvailable(googletest)
endif()

add_executable(aeb_tests tests/test_ttc.cpp tests/test_aeb_controller.cpp tests/test_scenarios.cpp)
target_link_libraries(aeb_tests PRIVATE aeb_sim GTest::gtest_main)
target_compile_options(aeb_tests PRIVATE -Wall -Wextra)

include(GoogleTest)
gtest_discover_tests(aeb_tests)
