cmake_minimum_required(VERSION 4.0.0)

set(CMAKE_CXX_COMPILER "C:\\MinGW\\bin\\mingw32-g++.exe")
set(CMAKE_C_COMPILER "C:\\MinGW\\bin\\mingw32-gcc.exe")

project(pleasehelp)


add_executable(${PROJECT_NAME} main.cpp)

include_directories(
    ${PROJECT_SOURCE_DIR}/eigen-3.4.0

)

target_link_libraries(${PROJECT_NAME} mylib)


add_subdirectory(lib eigen-3.4.0)