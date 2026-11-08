# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_controller C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ---------- Libraries ----------
add_library(lps331ap_i2c lps331ap_i2c/LPS.c)
target_include_directories(lps331ap_i2c PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/lps331ap_i2c
)
target_link_libraries(lps331ap_i2c PUBLIC
    pico_stdlib
    hardware_i2c
)

add_library(HWT906_TTL HWT906_TTL/jy901.c)
target_include_directories(HWT906_TTL PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/HWT906_TTL
)
target_link_libraries(HWT906_TTL PUBLIC
    pico_stdlib
    hardware_i2c
)

# ---------- micro-ROS ----------
link_directories(${CMAKE_CURRENT_LIST_DIR}/micro_ros_raspberrypi_pico_sdk/libmicroros)

# ---------- Main executable (must come before any target_* calls on it) ----------
add_executable(pico_controller
    pico_controller.c
    micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c
)

pico_set_program_name(pico_controller "pico_controller")
pico_set_program_version(pico_controller "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_controller 1)
pico_enable_stdio_usb(pico_controller 1)

# Includes
target_include_directories(pico_controller PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/micro_ros_raspberrypi_pico_sdk/libmicroros/include
)

# Link libraries
target_link_libraries(pico_controller
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_dma
    hardware_pio
    hardware_interp
    hardware_timer
    hardware_watchdog
    hardware_clocks
    lps331ap_i2c
    HWT906_TTL
    microros
)

# Optional but recommended for micro-ROS
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_add_extra_outputs(pico_controller)