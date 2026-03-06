from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch RViz2 with the scope_nav nav.rviz configuration."""

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('scope_nav'),
            'launch', 'rviz', 'nav.rviz'
        ),
        description='Full path to the RViz2 config file to use',
    )

    rviz_config = LaunchConfiguration('rviz_config')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='scope_nav_rviz',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
    ])
