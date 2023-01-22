# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/Users/roger/pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)
# TODO better for portability to submodule this
include("${PICO_SDK_PATH}/../lib/RF24/CMakeLists.txt")

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(nrf24joystick C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(nrf24joystick nrf24joystick.cpp)

pico_set_program_name(nrf24joystick "nrf24joystick")
pico_set_program_version(nrf24joystick "0.5")

pico_enable_stdio_uart(nrf24joystick 0)
pico_enable_stdio_usb(nrf24joystick 0)

# Add the standard library to the build
target_link_libraries(nrf24joystick
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(nrf24joystick PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(nrf24joystick
        hardware_spi
        hardware_sleep
        RF24
        )

pico_add_extra_outputs(nrf24joystick)
