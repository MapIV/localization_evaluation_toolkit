cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(height_evaluator)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (height_evaluator height_evaluator.cpp)
target_link_libraries (height_evaluator ${PCL_LIBRARIES})
