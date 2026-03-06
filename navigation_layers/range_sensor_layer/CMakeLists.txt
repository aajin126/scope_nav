cmake_minimum_required(VERSION 3.8)
project(range_sensor_layer)

# Compile options
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)

# ROS 2 deps
find_package(angles REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/range_sensor_layer.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  angles
  nav2_costmap_2d
  pluginlib
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# If your plugin uses pluginlib macros, this helps ensure plugin description is available.
# (You still need correct plugin XML content for Nav2.)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}
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

# ROS 2 python script install (replaces catkin_install_python)
install(
  PROGRAMS scripts/send_message.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  angles
  nav2_costmap_2d
  pluginlib
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()

