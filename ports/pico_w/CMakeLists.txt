# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# Raspberry Pi Pico W port (CYW43439): Classic HID gamepad and LE HID-over-GATT gamepad firmwares.
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

# --- the firmwares -------------------------------------------------------------------
# One board main (main.c) + one portable app. FLOSS_PICO_APP_LE selects the app:
#   floss_gamepad     Classic HID gamepad (examples/gamepad)
#   floss_le_gamepad  LE HID-over-GATT gamepad (examples/le_gamepad)
function(floss_pico_firmware name app_dir app_src app_le)
    add_executable(${name}
        main.c
        floss_hal_pico.c
        ${FLOSS_ROOT}/transport/cyw43/floss_cyw43.c
        ${app_src}
    )
    # Warnings only for our sources (SDK sources are compiled into this target too).
    set_source_files_properties(
        main.c
        floss_hal_pico.c
        ${FLOSS_ROOT}/transport/cyw43/floss_cyw43.c
        ${app_src}
        PROPERTIES COMPILE_OPTIONS "${FLOSS_PICO_WARN}")
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${FLOSS_ROOT}/src
        ${FLOSS_ROOT}/transport/cyw43
        ${app_dir}
    )
    target_compile_definitions(${name} PRIVATE
        CYW43_ENABLE_BLUETOOTH=1       # load the combined WLAN + radio firmware
        CYW43_LWIP=0                   # no IP stack
        # Never stall the radio for long on a slow / unread USB console.
        PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000
        FLOSS_PICO_SNOOP=$<BOOL:${FLOSS_PICO_SNOOP}>
        FLOSS_PICO_APP_LE=${app_le}
    )
    target_link_libraries(${name}
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
    pico_enable_stdio_usb(${name} 1)
    pico_enable_stdio_uart(${name} 0)
    pico_add_extra_outputs(${name})
endfunction()

floss_pico_firmware(floss_gamepad
    ${FLOSS_ROOT}/examples/gamepad ${FLOSS_ROOT}/examples/gamepad/gamepad_app.c 0)
floss_pico_firmware(floss_le_gamepad
    ${FLOSS_ROOT}/examples/le_gamepad ${FLOSS_ROOT}/examples/le_gamepad/le_gamepad_app.c 1)
