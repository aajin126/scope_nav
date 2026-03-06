from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

  scope_input_data_pub = Node(
    package='scope_nav',
    executable='scope_input_data_pub.py',
    name='scope_input_data_pub',
    output='screen',
  )

  return LaunchDescription([
    scope_input_data_pub,
  ])

