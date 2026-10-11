# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# Desktop build: library + unit/integration tests + host demo.
#   cmake -S . -B build -G Ninja && cmake --build build && ctest --test-dir build
#   add -DFLOSS_BUILD_FUZZ=ON for the sanitized fuzz harness (tests/fuzz).
# Embedded builds live in ports/<platform>/ (e.g. ports/pico_w).
cmake_minimum_required(VERSION 3.13)
option(FLOSS_BUILD_FUZZ "Build the sanitized fuzz harness (tests/fuzz) and register it with ctest" OFF)
if(FLOSS_BUILD_FUZZ AND POLICY CMP0091)
    # MSVC runtime selected per target: the fuzz target needs the release CRT for ASan.
    cmake_policy(SET CMP0091 NEW)
endif()
project(floss C)

option(FLOSS_BUILD_TESTS "Build the test suite" ON)
option(FLOSS_BUILD_POSIX "Build the desktop (posix/win32) port and demo" ON)
option(FLOSS_WERROR "Treat warnings as errors" ON)

include(cmake/floss.cmake)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set(FLOSS_WARN -Wall -Wextra -Wshadow -Wstrict-prototypes -Wno-unused-parameter)
    if(FLOSS_WERROR)
        list(APPEND FLOSS_WARN -Werror)
    endif()
    foreach(t floss floss_h4)
        target_compile_options(${t} PRIVATE ${FLOSS_WARN})
    endforeach()
    if(TARGET floss_sim)
        target_compile_options(floss_sim PRIVATE ${FLOSS_WARN})
    endif()
endif()
if(WIN32)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

if(FLOSS_BUILD_POSIX AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ports/posix/CMakeLists.txt")
    add_subdirectory(ports/posix)
endif()

if(FLOSS_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    enable_testing()
    add_subdirectory(tests)
endif()
