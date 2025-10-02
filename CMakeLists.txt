cmake_minimum_required(VERSION 3.20)
project(EvolutionaryFilter CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find ngspice shared library
find_library(NGSPICE_LIBRARY NAMES ngspice)
if(NOT NGSPICE_LIBRARY)
  message(FATAL_ERROR "libngspice not found. Install with: sudo apt install libngspice0-dev")
endif()

# Find ngspice headers
find_path(NGSPICE_INCLUDE_DIR ngspice/sharedspice.h)
if(NOT NGSPICE_INCLUDE_DIR)
  message(FATAL_ERROR "ngspice headers not found. Install with: sudo apt install libngspice0-dev")
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
  ${NGSPICE_INCLUDE_DIR}
)

# Threading support for parallel evaluation
find_package(Threads REQUIRED)
target_link_libraries(filter_evolver PRIVATE
  Threads::Threads
  ${NGSPICE_LIBRARY}
)

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
message(STATUS "NGSpice library: ${NGSPICE_LIBRARY}")
message(STATUS "NGSpice include: ${NGSPICE_INCLUDE_DIR}")
