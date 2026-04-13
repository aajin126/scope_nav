from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Gazebo Fake Localization Node
    fake_localization_node = Node(
        package='gazebo_fake_localization',
        executable='gazebo_fake_localization_node',
        name='gazebo_fake_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'use_odom': False},
            {'base_frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'model_name': 'burger'},
            {'freq': -1.0}
        ]
    )
    
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
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('predocc'),
                'launch', 'tb3', 'tb3_navigation.launch.py'))
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
        #static_tf,
        hunavsim_launch,
        nav_launch,
        #track_hunav,
        rviz_launch,
        scope_costmap_pub,
        #fake_localization_node,
    ])