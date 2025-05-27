from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
import os

def generate_launch_description():
    # Kill lingering Gazebo instances
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


    map_file = os.path.expanduser('/home/alec/ros2_ws/gallery_map.yaml')

    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')

    # Step 1: Launch Gazebo first
    gazebo_launch = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'turtlebot3_gazebo', 'Gallery_Test2.launch.py'
                ],
                output='screen'
            )
        ]
    )


    # Step 2: Launch Nav2 AFTER Gazebo has time to start (wait ~5s)
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                    f'map:={map_file}',
                    'use_sim_time:=true'
                ],
                output='screen'
            )
        ]
    )

    # Step 3: Launch other nodes after Nav2 (~5s delay)
    p_controller_launch = TimerAction(
        period=40.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'p_controller', 'p_controller'],
                output='screen'
            )
        ]
    )

    path_planner_launch = TimerAction(
        period=45.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'turtlebot_nav', 'path_to_goal4'],
                output='screen'
            )
        ]
    )

    template_match_launch = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'template_match', 'template_match'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        kill_zombies,
        set_model,
        gazebo_launch,
        nav2_launch,
        p_controller_launch,
        path_planner_launch,
        template_match_launch
    ])


#run this before the launch file
# killall -9 gzserver gzclient
# pkill -f map_server
# pkill -f nav2
# pkill -f rviz2
