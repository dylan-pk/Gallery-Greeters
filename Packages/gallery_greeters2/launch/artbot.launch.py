from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

import os

def generate_launch_description():
    simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='true',
        description='Whether to launch Gazebo simulation or real robot drivers'
    )
    use_simulation = LaunchConfiguration('use_simulation')

    kill_zombies = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'killall -9 gzserver gzclient || true; '
            'pkill -f map_server || true; '
            'pkill -f nav2 || true; '
            'pkill -f rviz2 || true'
        ],
        shell=True,
        output='screen'
    )

    sim_map_file = os.path.expanduser('/home/alec/ros2_ws/gallery_map.yaml')
    real_map_file = os.path.expanduser('/home/alec/ros2_ws/real_map5.yaml')

    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')

    gazebo_launch = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'turtlebot3_gazebo', 'Gallery_Test2.launch.py'
                ],
                output='screen',
                condition=IfCondition(use_simulation)
            )
        ]
    )

    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                    f'map:={sim_map_file}',
                    'use_sim_time:=true'
                ],
                output='screen',
                condition=IfCondition(use_simulation)
            )
        ]
    )

    nav2_real_launch = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                    'use_sim_time:=false',
                    f'map:={real_map_file}'
                ],
                output='screen',
                condition=UnlessCondition(use_simulation)
            )
        ]
    )

    # Fixed: p_controller in new terminal
    p_controller_launch = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run p_controller p_controller; exec bash'
                ],
                shell=False
            )
        ]
    )

    # Fixed: Path planner (sim)
    path_planner_launch_sim = TimerAction(
        period=30.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run turtlebot_nav path_to_goal5; exec bash'
                ],
                shell=False,
                condition=IfCondition(use_simulation)
            )
        ]
    )

    # Fixed: Path planner (real)
    path_planner_launch_real = TimerAction(
        period=30.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run turtlebot_nav path_to_goal4; exec bash'
                ],
                shell=False,
                condition=UnlessCondition(use_simulation)
            )
        ]
    )

    # Fixed: Template match node
    template_match_launch = TimerAction(
        period=11.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch template_match template_match_test.launch.py; exec bash'
                ],
                shell=False
            )
        ]
    )

    # Fixed: Audio interface node
    interface_launch = TimerAction(
        period=11.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run package_audio_messages speech_node; exec bash'
                ],
                shell=False
            )
        ]
    )

    return LaunchDescription([
        simulation_arg,
        kill_zombies,
        set_model,
        gazebo_launch,
        nav2_launch,
        nav2_real_launch,
        p_controller_launch,
        path_planner_launch_sim,
        path_planner_launch_real,
        template_match_launch,
        interface_launch
    ])



#run this before the launch file
# killall -9 gzserver gzclient
# pkill -f map_server
# pkill -f nav2
# pkill -f rviz2
