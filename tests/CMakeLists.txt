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
floss_add_test(test_l2cap    ${FLOSS_TEST_INTEGRATION})
floss_add_test(test_perf     ${FLOSS_TEST_INTEGRATION})

# The benchmark is informative; keep it out of quick default runs' critical path.
set_tests_properties(test_perf PROPERTIES LABELS perf)
