cmake_minimum_required(VERSION 3.8)
project(scope_nav)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY src/
    DESTINATION share/${PROJECT_NAME}/src
)

# Install Python nodes as executables
install(PROGRAMS
	src/scope_input_data_pub.py
	src/scope_costmap_data_pub.py
	src/scope_data_visualize_pub.py
	src/goal_visualize.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()

