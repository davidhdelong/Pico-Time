# This file is used to import the Pico SDK into your project.
#
# Set PICO_SDK_PATH in your environment or CMake cache to the Pico SDK
# directory, or add it to this file.

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif ()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH is not set. Set it to the Pico SDK location.")
endif ()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
