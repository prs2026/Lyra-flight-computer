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
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(TrilaterationTest C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(TrilaterationTest TrilaterationTest.cpp )

pico_set_program_name(TrilaterationTest "TrilaterationTest")
pico_set_program_version(TrilaterationTest "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(TrilaterationTest 0)
pico_enable_stdio_usb(TrilaterationTest 1)

# Add the standard library to the build
target_link_libraries(TrilaterationTest
        pico_stdlib
        
        )

# Add the standard include files to the build
target_include_directories(TrilaterationTest PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(TrilaterationTest PRIVATE /eigen-3.4.0)

include_directories(eigen)

add_library(eigen
       libraryname/include/dare/alloc.hpp
       
       )

pico_add_extra_outputs(TrilaterationTest)

#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/eigen-3.4.0)
