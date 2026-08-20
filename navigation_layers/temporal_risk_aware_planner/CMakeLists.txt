cmake_minimum_required(VERSION 3.10.2)
project(temporal_risk_aware_planner)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    vox_msgs
)

find_package(OpenMP)

generate_dynamic_reconfigure_options(
  cfg/TemporalRiskAwarePlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    vox_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/quadratic_calculator.cpp
  src/dijkstra.cpp
  src/astar.cpp
  src/grid_path.cpp
  src/gradient_path.cpp
  src/orientation_filter.cpp
  src/planner_core.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}
    OpenMP::OpenMP_CXX
  )
endif()

add_executable(temporal_risk_aware_planner_node
  src/plan_node.cpp
)

add_dependencies(temporal_risk_aware_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(temporal_risk_aware_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(temporal_risk_aware_planner_node
    OpenMP::OpenMP_CXX
  )
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS temporal_risk_aware_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
