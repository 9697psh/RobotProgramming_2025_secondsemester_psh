from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    output_filename_arg = DeclareLaunchArgument(
        'output_filename',
        default_value='default_day.json',
        description='Output json filename (e.g., 월요일.json)'
    )

    return LaunchDescription([
        output_filename_arg,

        Node(
            package='yolo_detector',
            executable='yolo_node',
            name='yolo_detector_node',
            output='screen',
        ),

        Node(
            package='my_project_pkg',
            executable='object_scanner',
            name='object_scanner_node',
            output='screen',
        ),

        Node(
            package='my_project_pkg',
            executable='json_logger',
            name='json_logger_node',
            output='screen',
            parameters=[{
                'output_filename': LaunchConfiguration('output_filename')
            }],
        ),

        Node(
            package='my_nav2_tools',
            executable='nav2_patrol',
            name='nav2_patrol_node',
            output='screen',
        ),
    ])
