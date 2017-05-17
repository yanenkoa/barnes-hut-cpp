cmake_minimum_required(VERSION 3.7)
project(BarnesHut)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES src/main.cpp)
add_subdirectory(eigen)
include_directories(eigen)
add_executable(BarnesHut ${SOURCE_FILES})