add_library(resources INTERFACE)
target_include_directories(resources
  INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/resources>)

add_library(gui STATIC
  gui.cpp
  guiplanner.cpp
  render.cpp
  scene.cpp
  )
target_include_directories(gui PUBLIC ${FOX_INCLUDE_DIR})
target_link_libraries(gui PUBLIC msl planner resources)

if (BUILD_GUI_GL)
  add_library(rendergl
    STATIC
    rendergl.cpp
    renderglobj.cpp
    )
  target_link_libraries(rendergl PUBLIC msl gui OpenGL::GL OpenGL::GLU GLUT::GLUT)

  add_executable(plangl plangl.cpp setup.cpp)
  target_include_directories(plangl PRIVATE ${FOX_INCLUDE_DIR})
  target_link_libraries(plangl PRIVATE msl gui rendergl ${FOX_LIBRARY})
endif()

if (BUILD_GUI_INVENTOR)
  add_library(renderiv
    STATIC
    renderiv.cpp
    )
  target_link_libraries(renderiv PUBLIC msl gui)

  add_executable(planiv planiv.cpp setup.cpp)
  target_include_directories(planiv PRIVATE ${FOX_INCLUDE_DIR})
  target_link_libraries(planiv PRIVATE msl gui renderiv ${FOX_LIBRARY})
endif()

if (BUILD_GUI_PERFORMER)
  add_executable(planpf planpf.cpp setup.cpp)
  target_include_directories(planpf PRIVATE ${FOX_INCLUDE_DIR})
  target_link_libraries(planpf PRIVATE msl gui ${FOX_LIBRARY})
endif()
