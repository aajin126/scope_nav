cmake_minimum_required(VERSION 3.8)
project(social_navigation_layers)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(people_msgs REQUIRED)

include_directories(include)

add_library(social_layers SHARED
  src/social_layer.cpp
  src/proxemic_layer.cpp
  src/passing_layer.cpp
)

target_include_directories(social_layers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(social_layers
  angles
  nav2_costmap_2d
  pluginlib
  rclcpp
  geometry_msgs
  people_msgs
)

# Export plugin description for Nav2 costmap plugins
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS social_layers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(social_layers)
ament_export_dependencies(
  angles
  nav2_costmap_2d
  pluginlib
  rclcpp
  geometry_msgs
  people_msgs
)

ament_package()

