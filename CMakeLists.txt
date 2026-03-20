cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Infantry_01)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Application/struct_typedef.h
        Application/robot_global.c
        Application/robot_global.h
        Bsp/can/bsp_can.c
        Bsp/can/bsp_can.h
        Bsp/LED/bsp_LED.c
        Bsp/LED/bsp_LED.h
        Bsp/uart/bsp_uart.c
        Bsp/uart/bsp_uart.h
        Components/ist8310_i2c/ist8310_driver.c
        Components/ist8310_i2c/ist8310_driver.h
        Application/auto_aim.c
        Application/auto_aim.h
        Bsp/usb_cdc/bsp_usb_cdc.c
        Bsp/usb_cdc/bsp_usb_cdc.h
        Components/ist8310_i2c/ist8310_middleware.c
        Components/ist8310_i2c/ist8310_middleware.h
        Application/auto_aim.c
        ALL_Task/send_to_referee_task.c
        ALL_Task/send_to_referee_task.h
        ALL_Task/send_to_C_board_task.c
        ALL_Task/send_to_C_board_task.h
        ALL_Task/analyze_from_referee_task.c
        ALL_Task/analyze_from_referee_task.h
        ALL_Task/analyze_from_supercapacitor_task.c
        ALL_Task/analyze_from_supercapacitor_task.h
        Application/referee.c
        Application/referee.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        -u _printf_float
)

