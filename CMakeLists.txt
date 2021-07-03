# CMake requirements
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(MotionStrategyLibrary VERSION 1.0)

# Define minimum C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define where to find custom CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Optional builds
include(CMakeDependentOption)
option(BUILD_GUI "Build planner applications" ON)
cmake_dependent_option(BUILD_GUI_GL "Build planner application using OpenGL" ON
                       "BUILD_GUI" OFF)
cmake_dependent_option(BUILD_GUI_INVENTOR "Build planner application using Inventor" OFF
                       "BUILD_GUI" OFF)
cmake_dependent_option(BUILD_GUI_PERFORMER "Build planner application using Performer" OFF
                       "BUILD_GUI" OFF)

# Find dependencies
find_package(PQP REQUIRED)

if (BUILD_GUI)
  find_package(FOX REQUIRED)

  if (BUILD_GUI_GL)
    set(OpenGL_GL_PREFERENCE GLVND)
    find_package(OpenGL REQUIRED)

    find_package(GLUT REQUIRED)
  endif()
endif()

# add sources directory
add_subdirectory(src)
