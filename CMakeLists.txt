cmake_minimum_required(VERSION 3.20)
project(EvolutionaryFilter CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find ngspice
find_program(NGSPICE_EXECUTABLE ngspice)
if(NOT NGSPICE_EXECUTABLE)
  message(WARNING "ngspice executable not found. Install with: sudo apt install ngspice")
endif()

# Source files - in this case, everything is in one file
# Split into multiple files as shown in the artifact structure
add_executable(filter_evolver
  src/main.cpp
  src/genome.cpp
  src/evolution.cpp
  src/fitness.cpp
  src/spice_interface.cpp
)

# Include directories
target_include_directories(filter_evolver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Threading support for parallel evaluation
find_package(Threads REQUIRED)
target_link_libraries(filter_evolver PRIVATE Threads::Threads)

# Optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_options(filter_evolver PRIVATE
    -O3
    -march=native
    -flto
  )
  target_link_options(filter_evolver PRIVATE
    -flto
  )
endif()

# Debug flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(filter_evolver PRIVATE
    -g
    -Wall
    -Wextra
    -pedantic
  )
endif()

# Print configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "NGSpice: ${NGSPICE_EXECUTABLE}")
