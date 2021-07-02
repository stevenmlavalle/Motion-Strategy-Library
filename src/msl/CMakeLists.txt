add_library(msl
  STATIC
  geom.cpp
  geom_pqp.cpp
  graph.cpp
  matrix.cpp
  model.cpp
  modelmisc.cpp
  model2d.cpp
  model3d.cpp
  modelcar.cpp
  nodeinfo.cpp
  point.cpp
  point3d.cpp
  polygon.cpp
  problem.cpp
  random.cpp
  solver.cpp
  tree.cpp
  triangle.cpp
  util.cpp
  vector.cpp
  )
target_include_directories(msl
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(msl PUBLIC ${PQP_LIBRARY})


add_library(planner
  STATIC
  fdp.cpp
  planner.cpp
  prm.cpp
  rcrrt.cpp
  rrt.cpp
  setup.cpp
  )
target_link_libraries(planner PUBLIC msl)

## GUI App

if (BUILD_GUI)
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
  target_link_libraries(gui PUBLIC planner resources)

  if (BUILD_GL_APP)
    add_library(rendergl
      STATIC
      rendergl.cpp
      renderglobj.cpp
      )
    target_link_libraries(rendergl PUBLIC msl gui OpenGL::GL OpenGL::GLU GLUT::GLUT)

    add_executable(plangl plangl.cpp)
    target_include_directories(plangl PRIVATE ${FOX_INCLUDE_DIR})
    target_link_libraries(plangl PRIVATE msl rendergl gui ${FOX_LIBRARY})
  endif()
endif()

# add_library(marray STATIC marray.cpp)
# target_link_libraries(marray PUBLIC msl)

# add_library(msllist STATIC msllist.cpp)
# target_link_libraries(msllist PUBLIC msl)

# add_library(planiv STATIC planiv.cpp)
# target_link_libraries(planiv PUBLIC msl)

# add_library(planpf STATIC planpf.cpp)
# target_link_libraries(planpf PUBLIC msl)


# add_library(renderiv STATIC renderiv.cpp)
# target_link_libraries(renderiv PUBLIC msl)
