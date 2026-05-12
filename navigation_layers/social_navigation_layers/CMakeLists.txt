cmake_minimum_required(VERSION 3.0.2)
project(social_navigation_layers)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  people_msgs
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP")
endif()

generate_dynamic_reconfigure_options(
  cfg/GridPredictionLayer.cfg
  cfg/ProxemicLayer.cfg
  cfg/ProbabilityLayer.cfg
)

catkin_package(
    CATKIN_DEPENDS angles costmap_2d dynamic_reconfigure geometry_msgs nav_msgs people_msgs pluginlib roscpp tf2 tf2_geometry_msgs tf2_ros
    INCLUDE_DIRS include
    LIBRARIES social_layers
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## add cpp library
add_library(social_layers
            src/social_layer.cpp
            src/proxemic_layer.cpp
            src/passing_layer.cpp
            src/probability_layer.cpp
            src/grid_prediction_layer.cpp
            src/crowd_layer.cpp
            src/utils.cpp
)
add_dependencies(social_layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    social_layers ${catkin_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(
    social_layers
    OpenMP::OpenMP_CXX
  )
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS social_layers
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
