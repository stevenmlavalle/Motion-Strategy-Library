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
  )
target_link_libraries(planner PUBLIC msl)
