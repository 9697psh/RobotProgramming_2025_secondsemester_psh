import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('clean_nav_pkg')
    nav2_bringup_pkg_share = get_package_share_directory('nav2_bringup')

    # Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map file to load')

    # Define Paths
    params_file_path = os.path.join(pkg_share, 'params', 'nav_params.yaml')
    rviz_config_path = os.path.join(nav2_bringup_pkg_share, 'rviz', 'nav2_default_view.rviz')

    # Define Nodes and Launch Includes
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file_path,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen')

    patrol_sender_node = Node(
        package='my_nav2_tools',
        executable='nav2_patrol',
        name='nav2_patrol',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        nav2_bringup_launch,
        rviz_node,
        patrol_sender_node,
    ])
