from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    gui = LaunchConfiguration('gui')
    statistics_file = LaunchConfiguration('statistics_file')
    model_file = LaunchConfiguration('model_file')
    rviz = LaunchConfiguration('rviz')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Bring up the Gazebo graphical interface')

    declare_statistics_file = DeclareLaunchArgument(
        'statistics_file',
        default_value=os.path.join(
            get_package_share_directory('scope_nav'),
            'src', 'model',
            'truncnorm_skewcauchy_statistics_tables',
            'truncnorm_skewcauchy_entropy_pred_time_6.npy'))

    declare_model_file = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(
            get_package_share_directory('scope_nav'),
            'src', 'model', 'model.pth'))

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true')

    hunavsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('predocc'),
                'launch', 'tb3', 'warehouse_tb3_hunav.launch.py'))
    )

    # navigation_AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('predocc'),
                'launch', 'tb3', 'tb3_navigation.launch.py'))
    )

    # Temporary fix: connect map and odom with an identity static TF
    # This ensures map -> odom -> base_footprint exists even if AMCL
    # does not publish the transform in this setup.
    # static_map_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_odom',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    #     output='screen',
    # )

    goal_visualize = Node(
        package='scope_nav',
        executable='goal_visualize',
        name='goal_visualize',
        output='screen'
    )

    scope_goal_visualize = Node(
        package='scope_nav',
        executable='goal_visualize',
        name='scope_goal_visualize',
        output='screen'
    )

    track_ped_pub = Node(
        package='scope_nav',
        executable='track_ped_pub',
        name='track_ped_pub',
        output='screen'
    )

    # SCOPE Costmap Publisher
    scope_costmap_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('scope_nav'),
                'launch', 'scope_costmap_pub.launch.py')),
        launch_arguments={
            'statistics_file': statistics_file,
            'model_file': model_file,
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('scope_nav'),
                'launch', 'nav_rviz.launch.py')),
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_gui,
        declare_statistics_file,
        declare_model_file,
        declare_rviz,
        hunavsim_launch,
        amcl_launch,
        #static_map_odom,
        #goal_visualize,
        #scope_goal_visualize,
        #track_ped_pub,
        scope_costmap_pub,
        rviz_launch,
    ])