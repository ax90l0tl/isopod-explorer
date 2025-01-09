cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(rov_micro C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

######################
## Lower Ebay node ##
######################

add_executable(lower_ebay
    src/lower_ebay.cpp
    src/pico_uart_transport.c
)

target_include_directories(lower_ebay PUBLIC
    include/${PROJECT_NAME}
    libmicroros/include
)

target_link_directories(lower_ebay PUBLIC libmicroros)

target_link_libraries(lower_ebay
    pico_stdlib
    microros
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(lower_ebay 1)
pico_enable_stdio_uart(lower_ebay 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(lower_ebay)

######################
## Upper Ebay node ##
######################

add_executable(upper_ebay
    src/upper_ebay.cpp
    src/pico_uart_transport.c
)
target_include_directories(upper_ebay PUBLIC
    include/${PROJECT_NAME}
    libmicroros/include
)

target_link_directories(upper_ebay PUBLIC libmicroros)

target_link_libraries(upper_ebay
    pico_stdlib
    microros
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(upper_ebay 1)
pico_enable_stdio_uart(upper_ebay 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(upper_ebay)
