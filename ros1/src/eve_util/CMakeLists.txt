cmake_minimum_required(VERSION 2.8.3)
project(eve_util)

# Enable support for C++14
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "{CMAKE_CXX_FLAGS} -std=c++14")
else ()
  set(CMAKE_CXX_STANDARD 14)
endif()

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  lanelet2_parser
)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  DEPENDS PCL
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(extract_within_lane
  src/extract_within_lane.cpp
)
target_link_libraries(extract_within_lane ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(extract_within_lane ${catkin_EXPORTED_TARGETS})
