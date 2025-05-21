from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch-time configurable parameters
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('namespace', default_value=''),

        # Smart camera node that auto-detects hardware
        Node(
            package='template_match',
            executable='camera_node',
            name='smart_cam_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen'
        ),

        # Template matching node
        Node(
            package='template_match',
            executable='template_match',
            name='template_matching_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic')
            }],
            output='screen'
        ),

        # Automatically launch RQT image viewer
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        )
    ])
