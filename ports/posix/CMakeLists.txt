# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# Desktop port (Windows + POSIX).
#   floss_port_posix    HAL (time, storage, log, entropy, critical) + serial stream
#   floss_gamepad_host  examples/gamepad over an H4 UART controller
#                       (only when the example sources are present)

add_library(floss_port_posix STATIC
    floss_hal_posix.c
    floss_serial.c
)
target_include_directories(floss_port_posix PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(floss_port_posix PUBLIC floss)
set_target_properties(floss_port_posix PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(floss_port_posix PUBLIC Threads::Threads)
endif()
if(DEFINED FLOSS_WARN)
    target_compile_options(floss_port_posix PRIVATE ${FLOSS_WARN})
endif()

set(FLOSS_GAMEPAD_DIR "${FLOSS_ROOT}/examples/gamepad")
if(EXISTS "${FLOSS_GAMEPAD_DIR}/gamepad_app.c" AND EXISTS "${FLOSS_GAMEPAD_DIR}/gamepad_app.h")
    add_executable(floss_gamepad_host
        main_gamepad.c
        "${FLOSS_GAMEPAD_DIR}/gamepad_app.c"
    )
    target_include_directories(floss_gamepad_host PRIVATE "${FLOSS_GAMEPAD_DIR}")
    # floss needs the HAL and the HAL lives in floss_port_posix: list the port
    # after floss so single-pass linkers (GNU ld) resolve the HAL symbols.
    target_link_libraries(floss_gamepad_host PRIVATE floss_h4 floss floss_port_posix)
    set_target_properties(floss_gamepad_host PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
    if(DEFINED FLOSS_WARN)
        set_source_files_properties(main_gamepad.c PROPERTIES COMPILE_OPTIONS "${FLOSS_WARN}")
    endif()
else()
    message(STATUS "FLOSS: examples/gamepad not found, skipping floss_gamepad_host")
endif()
