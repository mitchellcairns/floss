# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# Raspberry Pi Pico W port: Classic HID gamepad on the CYW43439.
#
#   cmake -G Ninja -S ports/pico_w -B build-pico
#   cmake --build build-pico
#   -> build-pico/floss_gamepad.uf2
#
# The CYW43 driver is used directly (pico_cyw43_driver + async_context_poll),
# NOT through pico_cyw43_arch, which would pull in another host stack.
cmake_minimum_required(VERSION 3.13)

# Log every HCI packet from boot as "SNOOP ..." lines (tools/floss_snoop.py
# turns them into a capture). Can also be toggled at runtime with 'p'.
option(FLOSS_PICO_SNOOP "Enable the HCI snoop log hook at boot" OFF)

if(NOT PICO_BOARD)
    set(PICO_BOARD pico_w CACHE STRING "Board type")
endif()

include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)

project(floss_pico_w C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

pico_sdk_init()

if(NOT PICO_CYW43_SUPPORTED)
    message(FATAL_ERROR "Board '${PICO_BOARD}' has no CYW43 radio (use pico_w or pico2_w)")
endif()

# --- the stack -------------------------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/floss.cmake)

target_compile_definitions(floss PUBLIC FLOSS_USER_CONFIG_FILE="floss_pico_config.h")
target_include_directories(floss PUBLIC ${CMAKE_CURRENT_LIST_DIR})
set(FLOSS_PICO_WARN -Wall -Wextra -Wshadow -Wno-unused-parameter)
target_compile_options(floss PRIVATE ${FLOSS_PICO_WARN})

# --- the firmware -------------------------------------------------------------------
add_executable(floss_gamepad
    main.c
    floss_hal_pico.c
    ${FLOSS_ROOT}/transport/cyw43/floss_cyw43.c
    ${FLOSS_ROOT}/examples/gamepad/gamepad_app.c
)

# Warnings only for our sources (SDK sources are compiled into this target too).
set_source_files_properties(
    main.c
    floss_hal_pico.c
    ${FLOSS_ROOT}/transport/cyw43/floss_cyw43.c
    ${FLOSS_ROOT}/examples/gamepad/gamepad_app.c
    PROPERTIES COMPILE_OPTIONS "${FLOSS_PICO_WARN}")

target_include_directories(floss_gamepad PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${FLOSS_ROOT}/src
    ${FLOSS_ROOT}/transport/cyw43
    ${FLOSS_ROOT}/examples/gamepad
)

target_compile_definitions(floss_gamepad PRIVATE
    CYW43_ENABLE_BLUETOOTH=1       # load the combined WLAN + radio firmware
    CYW43_LWIP=0                   # no IP stack
    # Never stall the radio for long on a slow / unread USB console.
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000
    FLOSS_PICO_SNOOP=$<BOOL:${FLOSS_PICO_SNOOP}>
)

target_link_libraries(floss_gamepad
    floss
    pico_stdlib
    pico_cyw43_driver
    cyw43_driver_picow
    pico_async_context_poll
    pico_unique_id
    hardware_flash
    hardware_sync
    pico_flash
    pico_rand
)

pico_enable_stdio_usb(floss_gamepad 1)
pico_enable_stdio_uart(floss_gamepad 0)
pico_add_extra_outputs(floss_gamepad)
