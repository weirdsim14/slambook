cmake_minimum_required(VERSION 3.10)
project(useEigen VERSION 1.0 LANGUAGES CXX)

# Set the build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Include Eigen headers
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add executable
add_executable(eigenMatrix eigenMatrix.cpp)

# Link Eigen
target_link_libraries(eigenMatrix Eigen3::Eigen)

# Print summary
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Eigen include directory: ${EIGEN3_INCLUDE_DIR}")
