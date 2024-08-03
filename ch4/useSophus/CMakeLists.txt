cmake_minimum_required(VERSION 3.10)
project(useSophus VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type to Release by default if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Find Sophus package
find_package(Sophus REQUIRED)

# Add the executable target
add_executable(useSophus useSophus.cpp)

# Link libraries
target_link_libraries(useSophus Sophus::Sophus)

# Include directories
target_include_directories(useSophus PRIVATE ${Sophus_INCLUDE_DIRS})

# Print project configuration summary
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Sophus include directories: ${Sophus_INCLUDE_DIRS}")
