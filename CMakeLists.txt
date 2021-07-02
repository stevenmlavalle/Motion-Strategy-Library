cmake_minimum_required(VERSION 3.10)

project(MotionStrategyLibrary VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(PQP REQUIRED)

find_package(FOX)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL)

find_package(GLUT)

if(OPENGL_FOUND AND GLUT_FOUND)
  set(BUILD_GL_APP TRUE)
else()
  set(BUILD_GL_APP FALSE)
endif()

if(FOX_FOUND AND BUILD_GL_APP)
  set(BUILD_GUI TRUE)
else()
  set(BUILD_GUI FALSE)
endif()

add_subdirectory(src/msl)
