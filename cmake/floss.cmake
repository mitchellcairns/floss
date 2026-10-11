# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# Include this file from any CMake project (desktop or embedded SDK) to get:
#   floss            - the stack (core, HCI, GAP, L2CAP, SDP, HID)
#   floss_h4         - H4 transport over a floss_stream_t
#   floss_sim        - virtual controller transport (tests / desktop demos)
# Platform transports needing a vendor SDK (e.g. CYW43) are defined by their
# port (see ports/pico_w/CMakeLists.txt).
#
# Configuration: target_compile_definitions(floss PUBLIC FLOSS_USER_CONFIG_FILE="...")

get_filename_component(FLOSS_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

file(GLOB_RECURSE FLOSS_SOURCES CONFIGURE_DEPENDS "${FLOSS_ROOT}/src/*.c")

add_library(floss STATIC ${FLOSS_SOURCES})
target_include_directories(floss PUBLIC "${FLOSS_ROOT}/include" PRIVATE "${FLOSS_ROOT}/src")
set_target_properties(floss PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

add_library(floss_h4 STATIC "${FLOSS_ROOT}/transport/h4/floss_h4.c")
target_include_directories(floss_h4 PUBLIC "${FLOSS_ROOT}/transport/h4")
target_link_libraries(floss_h4 PUBLIC floss)
set_target_properties(floss_h4 PROPERTIES C_STANDARD 11)

file(GLOB FLOSS_SIM_SOURCES CONFIGURE_DEPENDS "${FLOSS_ROOT}/transport/sim/*.c")
if(FLOSS_SIM_SOURCES)
    add_library(floss_sim STATIC ${FLOSS_SIM_SOURCES})
    target_include_directories(floss_sim PUBLIC "${FLOSS_ROOT}/transport/sim")
    target_link_libraries(floss_sim PUBLIC floss)
    set_target_properties(floss_sim PROPERTIES C_STANDARD 11)
endif()
