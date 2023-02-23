cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(color)

#SET(CMAKE_BUILD_TYPE "Debug”)
find_package(PCL 1.9 REQUIRED)
set(CMAKE_CXX_STANDARD 11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (color main)
target_link_libraries (color ${PCL_LIBRARIES})