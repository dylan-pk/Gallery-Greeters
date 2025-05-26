from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Arguments with TurtleBot-ready defaults
        DeclareLaunchArgument('image_transport', default_value='compressed'),  # ✅ compressed by default
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw/compressed'),  # ✅ from TurtleBot
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_local_camera', default_value='false'),  # ✅ default: TurtleBot mode

        # Only launch local camera if testing on laptop
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'format': 'MJPEG'},
                {'width': 640},
                {'height': 480}
            ],
            condition=IfCondition(LaunchConfiguration('use_local_camera'))  # ✅ Only runs when testing locally
        ),

        # Template matching node
        Node(
            package='template_match',
            executable='template_match',
            name='template_matching_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'image_topic': LaunchConfiguration('image_topic')},
                {'image_transport': LaunchConfiguration('image_transport')},
                {'confidence_threshold': 0.65}
            ],
            output='screen'
        ),

        # Optional RQT viewer for visual debug
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        )
    ])
