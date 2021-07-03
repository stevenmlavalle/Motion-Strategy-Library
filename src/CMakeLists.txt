# MSL library
add_subdirectory(msl)

# MSL apps
if (BUILD_GUI)
  add_subdirectory(msl_gui)
endif()
