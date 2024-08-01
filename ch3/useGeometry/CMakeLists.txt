cmake_minimum_required(VERSION 3.10)
project(geometry VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type to Release by default if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Find Eigen3 package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add the executable target
add_executable(eigenGeometry eigenGeometry.cpp)

# Link Eigen3 to the target
target_link_libraries(eigenGeometry Eigen3::Eigen)

# Include Eigen3 directories
target_include_directories(eigenGeometry PRIVATE ${EIGEN3_INCLUDE_DIR})

# Print project configuration summary
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
