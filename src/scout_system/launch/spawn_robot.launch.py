"""Spawn the TurtleBot3 (burger_cam_pinhole) at the dock pose.

We diverge from the stock turtlebot3_gazebo `spawn_turtlebot3.launch.py`
because the upstream burger_cam SDF declares a 182 deg `wideanglecamera`
with a custom non-linear lens. That projection cannot be encoded in
the standard 3x3 pinhole K matrix that gazebo_ros_camera publishes on
/camera/camera_info, so the plugin emits garbage (fx=fy=-3.3 in
practice) and any pinhole-based perception (atan2(u-cx, fx) bearings,
homographies, etc.) is meaningless.

Instead, we point spawn_entity.py at our local SDF
(`scout_system/models/turtlebot3_burger_cam_pinhole/model.sdf`),
which is identical to the upstream model except the camera is a
normal 62.2 deg pinhole matching the actual Raspberry Pi camera v2
on the real robot. The robot_state_publisher still uses the upstream
URDF (frame names are unchanged, so all TFs line up).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_scout = get_package_share_directory('scout_system')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    x = DeclareLaunchArgument('x_pose', default_value='-2.6')
    y = DeclareLaunchArgument('y_pose', default_value='-3.0')
    yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Our pinhole-camera SDF, NOT the upstream wide-angle one.
    sdf_path = os.path.join(
        pkg_scout, 'models', 'turtlebot3_burger_cam_pinhole', 'model.sdf'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_burger_cam_pinhole',
        arguments=[
            # Keep the entity name as the upstream "burger" so any
            # Gazebo plugins/topics that key off it (e.g. namespace
            # conventions in turtlebot3_gazebo plugins) don't change.
            '-entity', 'burger',
            '-file', sdf_path,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.01',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    # robot_state_publisher still uses the upstream URDF -- frames
    # (base_link, base_scan, camera_link, camera_rgb_*) are unchanged.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([x, y, yaw, use_sim_time, rsp, spawn])
