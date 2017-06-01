cmake_minimum_required(VERSION 3.7)
project(BarnesHut)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find SFML
find_package(SFML REQUIRED COMPONENTS graphics window system)

if(NOT SFML_FOUND)
    message(FATAL_ERROR "SFML couldn't be located!")
endif()

include_directories(${SFML_INCLUDE_DIR})

find_package(Threads REQUIRED)

set(SOURCE_FILES src/main.cpp)
add_subdirectory(eigen)
include_directories(eigen)
add_executable(BarnesHut ${SOURCE_FILES})
target_link_libraries(BarnesHut ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
target_link_libraries(BarnesHut Threads::Threads)
