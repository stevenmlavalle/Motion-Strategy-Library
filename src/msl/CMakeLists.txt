add_library(msl_include INTERFACE)
target_include_directories(msl_include
  INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

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
target_link_libraries(msl PUBLIC msl_include ${PQP_LIBRARY})

add_library(planner
  STATIC
  fdp.cpp
  planner.cpp
  prm.cpp
  rcrrt.cpp
  rrt.cpp
  )
target_link_libraries(planner PUBLIC msl)
