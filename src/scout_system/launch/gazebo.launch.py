"""Bring up gzserver + gzclient with the scout arena world.

Intentionally small: takes a ``world`` launch arg (the filename in
``scout_system/worlds/``) so the same file can drive alternate worlds
later without editing Python.

Also prepends our ``models/`` directory to ``GAZEBO_MODEL_PATH`` so the
``<include><uri>model://hazard_cube_red</uri>`` style references inside
the .world file resolve.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_scout = get_package_share_directory('scout_system')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    world_arg = DeclareLaunchArgument(
        'world', default_value='scout_arena.world',
        description='World filename inside scout_system/worlds/',
    )

    world_path = PathJoinSubstitution([pkg_scout, 'worlds', LaunchConfiguration('world')])

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    our_models = os.path.join(pkg_scout, 'models')
    combined_model_path = (
        our_models if not existing_model_path else f'{our_models}:{existing_model_path}'
    )

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=combined_model_path
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '-s libgazebo_ros_state.so',
        }.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py')
        )
    )

    return LaunchDescription([world_arg, set_model_path, gz_server, gz_client])
