cmake_minimum_required(VERSION 3.7)
project(BarnesHut)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)

# Find SFML
find_package(SFML REQUIRED COMPONENTS graphics window system)

if(NOT SFML_FOUND)
    message(FATAL_ERROR "SFML couldn't be located!")
endif()

include_directories(${SFML_INCLUDE_DIR})

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(SOURCE_FILES src/main.cpp)
add_subdirectory(eigen)
include_directories(eigen)
add_executable(BarnesHut ${SOURCE_FILES})
target_link_libraries(BarnesHut ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
