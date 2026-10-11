# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# Raspberry Pi Pico W port (CYW43439): Classic HID gamepad and LE HID-over-GATT gamepad firmwares,
# plus the pad <-> dongle link pair (floss_link_pad, floss_link_dongle).
#
#   cmake -G Ninja -S ports/pico_w -B build-pico
#   cmake --build build-pico
#   -> build-pico/floss_gamepad.uf2, floss_le_gamepad.uf2, floss_link_pad.uf2, floss_link_dongle.uf2
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

set(FLOSS_GAMEPAD_SOURCES
    ${FLOSS_ROOT}/examples/gamepad/gamepad_app.c
    ${FLOSS_ROOT}/examples/gamepad/gamepad_state.c)
floss_pico_firmware(floss_gamepad
    ${FLOSS_ROOT}/examples/gamepad "${FLOSS_GAMEPAD_SOURCES}" 0)
floss_pico_firmware(floss_le_gamepad
    ${FLOSS_ROOT}/examples/le_gamepad ${FLOSS_ROOT}/examples/le_gamepad/le_gamepad_app.c 1)

# --- pad <-> dongle link firmwares (floss_link, sources in link/) ----------------
#   floss_link_pad     the controller: gamepad state -> floss_link_set_input()
#   floss_link_dongle  the USB receiver: floss_link -> USB HID gamepad + CDC console
# Both share the board bring-up (link/link_board.c), the HAL, the CYW43
# transport and the gamepad report format (examples/gamepad/gamepad_state.c).
#
# USB_DEVICE: the firmware runs its own TinyUSB device (tusb_config.h,
# descriptors and a CDC stdio driver in link/) instead of the SDK's
# pico_stdio_usb, which would bring its own descriptors.
function(floss_pico_link_firmware name main_src)
    cmake_parse_arguments(ARG "USB_DEVICE" "" "SOURCES" ${ARGN})
    set(srcs
        link/${main_src}
        link/link_board.c
        link/link_diag.c
        floss_hal_pico.c
        ${FLOSS_ROOT}/transport/cyw43/floss_cyw43.c
        ${FLOSS_ROOT}/examples/gamepad/gamepad_state.c
        ${ARG_SOURCES}
    )
    add_executable(${name} ${srcs})
    # Warnings only for our sources (SDK / TinyUSB sources compile into this target too).
    set_source_files_properties(${srcs} PROPERTIES COMPILE_OPTIONS "${FLOSS_PICO_WARN}")
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${FLOSS_ROOT}/transport/cyw43
        ${FLOSS_ROOT}/examples/gamepad
    )
    target_compile_definitions(${name} PRIVATE
        CYW43_ENABLE_BLUETOOTH=1       # load the combined WLAN + radio firmware
        CYW43_LWIP=0                   # no IP stack
        FLOSS_PICO_SNOOP=$<BOOL:${FLOSS_PICO_SNOOP}>
    )
    target_link_libraries(${name}
        floss
        pico_stdlib
        pico_cyw43_driver
        cyw43_driver_picow
        pico_async_context_poll
        pico_unique_id
        pico_bootrom
        hardware_flash
        hardware_sync
        hardware_watchdog
        pico_flash
        pico_rand
    )
    if(ARG_USB_DEVICE)
        # link/tusb_config.h configures TinyUSB. Only on this include path:
        # the SDK's pico_stdio_usb (pad) must keep finding its own.
        target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/link)
        target_link_libraries(${name} tinyusb_device tinyusb_board)
        pico_enable_stdio_usb(${name} 0)
    else()
        # Never stall the radio for long on a slow / unread USB console.
        target_compile_definitions(${name} PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000)
        pico_enable_stdio_usb(${name} 1)
    endif()
    pico_enable_stdio_uart(${name} 0)
    pico_add_extra_outputs(${name})
endfunction()

floss_pico_link_firmware(floss_link_pad main_pad.c)
floss_pico_link_firmware(floss_link_dongle main_dongle.c USB_DEVICE SOURCES
    link/usb_stdio.c
    link/usb_descriptors.c
    link/usb_gamepad.c
    link/link_meter.c
)
