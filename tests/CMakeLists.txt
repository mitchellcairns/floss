# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# Test suites. One executable per suite, each linked against the stack, the
# H4 and sim transports, and the deterministic test HAL (tests/test_hal.c -
# NOT ports/posix: tests own the clock, storage and randomness).
#
#   cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
#   FLOSS_TEST_QUIET=1 ctest ...        # silence stack logs
#   build/tests/test_hid_flow reconnect # run tests whose name contains "reconnect"

if(NOT TARGET floss_sim)
    message(FATAL_ERROR "tests need the floss_sim transport (transport/sim)")
endif()

# Framework + HAL are compiled into every executable (object files, so the
# HAL symbols the static floss library needs always resolve, whatever the
# linker's archive ordering rules).
set(FLOSS_TEST_BASE
    ${CMAKE_CURRENT_SOURCE_DIR}/floss_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_hal.c)
set(FLOSS_TEST_INTEGRATION
    ${FLOSS_TEST_BASE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_common.c)

function(floss_add_test name)
    add_executable(${name} ${name}.c ${ARGN})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${FLOSS_ROOT}/src)
    target_link_libraries(${name} PRIVATE floss_sim floss_h4 floss)
    set_target_properties(${name} PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${name} PRIVATE -Wall -Wextra -Wno-unused-parameter -Werror)
    elseif(MSVC)
        target_compile_options(${name} PRIVATE /W3 /WX)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

floss_add_test(test_core     ${FLOSS_TEST_BASE})
floss_add_test(test_h4       ${FLOSS_TEST_BASE})
floss_add_test(test_sdp      ${FLOSS_TEST_BASE})
floss_add_test(test_hid_flow ${FLOSS_TEST_INTEGRATION})
floss_add_test(test_hid_boot ${FLOSS_TEST_INTEGRATION})
floss_add_test(test_hid_integration ${FLOSS_TEST_INTEGRATION})
floss_add_test(test_l2cap    ${FLOSS_TEST_INTEGRATION})
floss_add_test(test_perf     ${FLOSS_TEST_INTEGRATION})
floss_add_test(test_le_central ${FLOSS_TEST_INTEGRATION})
floss_add_test(test_link     ${FLOSS_TEST_INTEGRATION})

# The benchmark is informative; keep it out of quick default runs' critical path.
set_tests_properties(test_perf PROPERTIES LABELS perf)

# ---------------------------------------------------------------------------
# Fuzz harness (-DFLOSS_BUILD_FUZZ=ON). The whole stack, the sim and the test
# framework are compiled straight into one executable so every object gets
# the sanitizers; the regular floss library and suites are left untouched.
#   build/tests/floss_fuzz [--seed N] [--iters N] [--verbose] [target]
# ---------------------------------------------------------------------------
if(FLOSS_BUILD_FUZZ)
    set(FLOSS_FUZZ_SAN_FLAGS "")
    if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU" AND NOT MSVC) # GCC-style driver (not cl / clang-cl)
        include(CheckCSourceCompiles)
        set(CMAKE_REQUIRED_FLAGS "-fsanitize=address,undefined")
        set(CMAKE_REQUIRED_LINK_OPTIONS "-fsanitize=address,undefined")
        if(WIN32)
            # ASan on Windows needs the release static CRT (no debug CRT).
            set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")
        endif()
        check_c_source_compiles("int main(void) { return 0; }" FLOSS_HAVE_ASAN_UBSAN)
        unset(CMAKE_MSVC_RUNTIME_LIBRARY)
        unset(CMAKE_REQUIRED_FLAGS)
        unset(CMAKE_REQUIRED_LINK_OPTIONS)
        if(FLOSS_HAVE_ASAN_UBSAN)
            set(FLOSS_FUZZ_SAN_FLAGS -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
        else()
            message(WARNING "floss_fuzz: toolchain lacks -fsanitize=address,undefined; building without sanitizers")
        endif()
    endif()

    add_executable(floss_fuzz
        ${CMAKE_CURRENT_SOURCE_DIR}/fuzz/floss_fuzz.c
        ${FLOSS_TEST_INTEGRATION}
        ${FLOSS_SOURCES}
        ${FLOSS_SIM_SOURCES}
        ${FLOSS_ROOT}/transport/h4/floss_h4.c)
    target_include_directories(floss_fuzz PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${FLOSS_ROOT}/include ${FLOSS_ROOT}/src
        ${FLOSS_ROOT}/transport/sim ${FLOSS_ROOT}/transport/h4)
    set_target_properties(floss_fuzz PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    if(WIN32 AND FLOSS_FUZZ_SAN_FLAGS)
        set_target_properties(floss_fuzz PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreaded")
    endif()
    if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU" AND NOT MSVC)
        target_compile_options(floss_fuzz PRIVATE -Wall -Wextra -Wno-unused-parameter -Werror -g ${FLOSS_FUZZ_SAN_FLAGS})
        target_link_options(floss_fuzz PRIVATE ${FLOSS_FUZZ_SAN_FLAGS})
    endif()
    add_test(NAME floss_fuzz COMMAND floss_fuzz --seed 1 --iters 20000)
    set_tests_properties(floss_fuzz PROPERTIES LABELS fuzz TIMEOUT 300 ENVIRONMENT "FLOSS_TEST_QUIET=1")
endif()
