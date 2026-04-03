from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    scope_nav_share = get_package_share_directory('scope_nav')

    statistics_file = LaunchConfiguration('statistics_file')
    model_file = LaunchConfiguration('model_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from /clock'
    )

    declare_statistics_file = DeclareLaunchArgument(
        'statistics_file',
        default_value=os.path.join(
            scope_nav_share,
            'src', 'model',
            'truncnorm_skewcauchy_statistics_tables',
            'truncnorm_skewcauchy_entropy_pred_time_6.npy'
        ),
        description='Path to statistics file'
    )

    declare_model_file = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(
            scope_nav_share,
            'src', 'model',
            'model_90.pth'
        ),
        description='Path to SO-SCOPE model file'
    )

    # SCOPE INPUT DATA Publisher 
    scope_input_data_pub = Node(
        package='scope_nav',
        executable='scope_input_data_pub.py',
        name='scope_input_data_pub',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # SCOPE Costmap Publisher
    scope_costmap_data_pub = Node(
        package='scope_nav',
        executable='scope_costmap_data_pub.py',
        name='scope_costmap_data_pub',
        output='screen',
        parameters=[{
            'statistics_file': statistics_file,
            'model_file': model_file,
            'use_sim_time': use_sim_time,
        }]
    )

    # SCOPE DATA Visualizer Publisher
    scope_data_visualize_pub = Node(
        package='scope_nav',
        executable='scope_data_visualize_pub.py',
        name='scope_data_visualize_pub',
        output='screen',
    )

    return LaunchDescription([
        declare_statistics_file,
        declare_model_file,
        declare_use_sim_time,
        scope_input_data_pub,
        scope_costmap_data_pub,
        #scope_data_visualize_pub,
    ])