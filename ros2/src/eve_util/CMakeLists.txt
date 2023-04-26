cmake_minimum_required(VERSION 3.8)
project(eve_util)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set the C++ compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)

if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_STANDARD 17)
  message(STATUS "Using C++17 standard.")
elseif(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_STANDARD 14)
  message(STATUS "Using C++14 standard.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or above support. Please use a different C++ compiler.")
endif()

if(CMAKE_BUILD_TYPE MATCHES "Release")
  # Set "Show Warnings", "Enable Compilation Optimization"
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

  # Set "Optimize the code according to the specified architecture instructions"
  # Note: if the CPU does not recognize the corresponding instruction, resulting in segmentation fault or illegal instruction.
  option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
  if(BUILD_WITH_MARCH_NATIVE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
    message(STATUS "Build with -march=native.")
  endif()
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
  # Set "Show Warnings", "Enable Compilation Optimization", "Contains gdb debug information"
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O0 -g")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g")

  # Set "Enable code coverage"
  option(BUILD_CODE_COVERAGE "Enable code coverage (Linux builds only)" OFF)
  if(BUILD_CODE_COVERAGE and not WIN32 and not APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
    message(STATUS "Enable code coverage.")
  endif()
endif()

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Boost
find_package(Boost REQUIRED)      
message(STATUS "Boost version: ${Boost_VERSION}")

# Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")

# PCL
find_package(PCL REQUIRED)
message(STATUS "PCL version: ${PCL_VERSION}")

# OpenMP
option(USE_OpenMP "Use OpenMP" ON)
if(USE_OpenMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "Using OpenMP.")
  endif()
endif()

# CUDA
option(USE_CUDA "Use CUDA" ON)
if(USE_CUDA)
  find_package(CUDA)
  if(CUDA_FOUND)
    message(STATUS "CUDA version: ${CUDA_VERSION} "
        "${CUDA_LIBRARIES};"
        "${CUDA_cusolver_LIBRARY};"
        "${CUDA_cusparse_LIBRARY}")

    if(NOT DEFINED CMAKE_CUDA_STANDARD)
      set(CMAKE_CUDA_STANDARD 14)
      set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
      message(STATUS "Using CUDA 14 standard.")
    endif()
  endif()
endif()


ament_auto_add_executable(trajectory_viewer src/trajectory_viewer.cpp)
target_include_directories(trajectory_viewer PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(trajectory_viewer
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)
install(TARGETS trajectory_viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ament_auto_add_executable(height_evaluator src/height_evaluator.cpp)
target_include_directories(height_evaluator PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(height_evaluator
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)
install(TARGETS height_evaluator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ament_auto_add_executable(scan2pcd src/scan2pcd.cpp)
target_include_directories(scan2pcd PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan2pcd
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)
install(TARGETS scan2pcd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


ament_auto_package()
