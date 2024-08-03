cmake_minimum_required(VERSION 3.10)
project(visualizeGeometry VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type to Release by default if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Find Eigen3 package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find Pangolin package
find_package(Pangolin REQUIRED)

# Add the executable target
add_executable(visualizeGeometry visualizeGeometry.cpp)

# Link libraries
target_link_libraries(visualizeGeometry Eigen3::Eigen ${Pangolin_LIBRARIES})

# Include directories
target_include_directories(visualizeGeometry PRIVATE ${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS})

# Print project configuration summary
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Pangolin include directories: ${Pangolin_INCLUDE_DIRS}")
