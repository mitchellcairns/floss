# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# Locate the Raspberry Pi Pico SDK and its host tools, then load it.
# Include this BEFORE project().
#
# Search order for each item:
#   1. -D<VAR>=... on the cmake command line (or the CMake cache)
#   2. the environment variable of the same name
#   3. the layout installed by the Raspberry Pi Pico VS Code extension:
#        ~/.pico-sdk/sdk/<FLOSS_PICO_SDK_VERSION>
#        ~/.pico-sdk/toolchain/<FLOSS_PICO_TOOLCHAIN_VERSION>
#        ~/.pico-sdk/tools/<FLOSS_PICO_SDK_VERSION>/pioasm
#        ~/.pico-sdk/picotool/<FLOSS_PICO_SDK_VERSION>/picotool
#
# Using the prebuilt pioasm/picotool avoids the SDK trying to compile them
# from source, which needs a native host compiler.

set(FLOSS_PICO_SDK_VERSION "2.2.0" CACHE STRING "Pico SDK version to look for under ~/.pico-sdk")
set(FLOSS_PICO_TOOLCHAIN_VERSION "14_2_Rel1" CACHE STRING "ARM toolchain version to look for under ~/.pico-sdk")

if(DEFINED ENV{USERPROFILE})
    file(TO_CMAKE_PATH "$ENV{USERPROFILE}" _floss_home)
else()
    set(_floss_home "$ENV{HOME}")
endif()
set(_floss_pico_root "${_floss_home}/.pico-sdk")

# --- SDK ---------------------------------------------------------------------
if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    file(TO_CMAKE_PATH "$ENV{PICO_SDK_PATH}" PICO_SDK_PATH)
endif()
if(NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH "${_floss_pico_root}/sdk/${FLOSS_PICO_SDK_VERSION}")
endif()
get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH)
if(NOT EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
    message(FATAL_ERROR "Pico SDK not found at '${PICO_SDK_PATH}'. Pass -DPICO_SDK_PATH=... or set the environment variable.")
endif()
set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to the Raspberry Pi Pico SDK" FORCE)

# --- ARM toolchain -------------------------------------------------------------
if(NOT PICO_TOOLCHAIN_PATH AND NOT DEFINED ENV{PICO_TOOLCHAIN_PATH})
    set(_floss_tc "${_floss_pico_root}/toolchain/${FLOSS_PICO_TOOLCHAIN_VERSION}")
    if(EXISTS "${_floss_tc}/bin")
        set(PICO_TOOLCHAIN_PATH "${_floss_tc}")
    endif()
endif()

# --- prebuilt host tools ---------------------------------------------------------
if(NOT pioasm_DIR AND EXISTS "${_floss_pico_root}/tools/${FLOSS_PICO_SDK_VERSION}/pioasm")
    set(pioasm_DIR "${_floss_pico_root}/tools/${FLOSS_PICO_SDK_VERSION}/pioasm")
endif()
if(NOT picotool_DIR AND EXISTS "${_floss_pico_root}/picotool/${FLOSS_PICO_SDK_VERSION}/picotool")
    set(picotool_DIR "${_floss_pico_root}/picotool/${FLOSS_PICO_SDK_VERSION}/picotool")
endif()

include("${PICO_SDK_PATH}/pico_sdk_init.cmake")
