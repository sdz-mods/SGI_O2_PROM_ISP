# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# Custom board for RP2354B (RP2350B package, 48 GPIOs)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)
set(PICO_BOARD     o2_prom_isp  CACHE STRING "Board type"     FORCE)
set(PICO_PLATFORM  rp2350-arm-s CACHE STRING "Pico platform"  FORCE)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(O2_PROM_ISP C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable
add_executable(O2_PROM_ISP O2_PROM_ISP.c)

pico_set_program_name(O2_PROM_ISP "O2_PROM_ISP")
pico_set_program_version(O2_PROM_ISP "A00")

# USB CDC for communication - no UART
pico_enable_stdio_uart(O2_PROM_ISP 0)
pico_enable_stdio_usb(O2_PROM_ISP 1)

# Add the standard library and hardware GPIO
target_link_libraries(O2_PROM_ISP
        pico_stdlib
        hardware_gpio
        pico_bootrom
)

target_include_directories(O2_PROM_ISP PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(O2_PROM_ISP)
