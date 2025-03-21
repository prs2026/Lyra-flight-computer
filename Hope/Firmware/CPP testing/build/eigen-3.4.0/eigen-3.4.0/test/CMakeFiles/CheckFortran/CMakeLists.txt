cmake_minimum_required(VERSION 4.0.0-rc4)
set(CMAKE_MODULE_PATH "C:/Users/E/Documents/GitHub/Lyra-flight-computer/Hope/Firmware/CPP testing/learninghelp/lib/eigen-3.4.0/cmake")
project(CheckFortran Fortran)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/result.cmake"
  "set(CMAKE_Fortran_COMPILER \"${CMAKE_Fortran_COMPILER}\")\n"
  
  )