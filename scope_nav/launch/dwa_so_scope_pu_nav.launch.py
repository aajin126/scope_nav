from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gazebo_map_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'gazebo'],
            output='screen'
        )

    gui = LaunchConfiguration('gui')
    statistics_file = LaunchConfiguration('statistics_file')
    model_file = LaunchConfiguration('model_file')
    rviz = LaunchConfiguration('rviz')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Bring up the Gazebo graphical interface')

    declare_model_file = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(
            get_package_share_directory('scope_nav'),
            'src', 'model', 'predocc_vae', 'v1.6','model.pth'))

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

    track_hunav = Node(
        package='scope_nav',
        executable='track_hunav.py',
        name='track_hunav',
        output='screen',
        parameters=[{
            'input_topic': '/people',
            'output_topic': '/people_viz',
            'base_frame': 'base_footprint',
        }]
    )

    # SCOPE Costmap Publisher
    scope_costmap_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('scope_nav'),
                'launch', 'scope_costmap_pub.launch.py')),
        launch_arguments={
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
        declare_model_file,
        declare_rviz,
        static_tf,
        hunavsim_launch,
        amcl_launch,
        #track_hunav,
        rviz_launch,
        scope_costmap_pub,
    ])