"""Spawn the TurtleBot3 (burger_cam by default) at the dock pose.

Thin wrapper around the stock ``turtlebot3_gazebo`` launch files so we
get the correct model.sdf + robot_state_publisher in one shot. The
caller controls the spawn pose via launch arguments.

Requires the TURTLEBOT3_MODEL env var to be set (burger_cam recommended
for the scout). The upstream launch file reads it directly.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    x = DeclareLaunchArgument('x_pose', default_value='-2.6')
    y = DeclareLaunchArgument('y_pose', default_value='-3.0')
    yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items(),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([x, y, yaw, use_sim_time, rsp, spawn])
