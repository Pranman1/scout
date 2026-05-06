"""Scout system launcher.

Single entry point for every demo mode, same pattern as the grazen
system.launch.py but reorganized for the hazard-detection flow.

Arguments
---------
mode       : 'sim' | 'real'    (default 'sim')
task       : 'scout' | 'map' | 'nav' | 'mission'
             - scout   : map + detect hazards, then run the mission (full demo)
             - map     : mapping only (auto_map toggles explore_lite vs WASD)
             - nav     : load a saved map, run Nav2 + AMCL (no mission manager)
             - mission : load saved map + saved hazards + run mission_manager
auto_map   : 'true' | 'false'  (only used in task=map; picks explore_lite vs manual)
map_name   : string            (filename stem under scout_system/maps/; auto if empty)
world      : world filename    (default 'scout_arena.world')

Notes on mode=real
------------------
The scout_ws is intended to overlay on an underlay workspace that
provides the stock turtlebot3 bringup (typically ~/turtle_test). Source
both before launching::

    source ~/turtle_test/install/setup.bash
    source ~/scout_ws/install/setup.bash

In real mode this launch file does NOT bring up the robot base -- run
``ros2 launch turtlebot3_bringup robot.launch.py`` on the Pi yourself.
Only nav-stack / perception / mission pieces launch here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetRemap


def _cond(expr):
    """Build an IfCondition from a list of substitutions/strings.

    Callers pass a list (e.g. ``sim_ex + [' and '] + needs_slam_ex``);
    PythonExpression wants that list directly, not wrapped in another.
    """
    return IfCondition(PythonExpression(expr))


def generate_launch_description():
    pkg_scout = get_package_share_directory('scout_system')
    pkg_tb3_nav = get_package_share_directory('turtlebot3_navigation2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # --- args ---------------------------------------------------------
    mode_arg = DeclareLaunchArgument('mode', default_value='sim')
    task_arg = DeclareLaunchArgument('task', default_value='scout')
    auto_map_arg = DeclareLaunchArgument('auto_map', default_value='true')
    world_arg = DeclareLaunchArgument('world', default_value='scout_arena.world')
    map_name_arg = DeclareLaunchArgument('map_name', default_value='')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')

    mode = LaunchConfiguration('mode')
    task = LaunchConfiguration('task')
    auto_map = LaunchConfiguration('auto_map')
    world = LaunchConfiguration('world')
    map_name_input = LaunchConfiguration('map_name')

    # auto map name: provided > world_stem > 'real_map'.
    # `world` may come in as either a bare filename ('scout_arena.world') or
    # an absolute path; rsplit('/', 1)[-1] grabs just the filename portion so
    # the resulting map name stays sane ('sim_scout_arena' either way).
    final_map_name = PythonExpression([
        "'", map_name_input, "' if '", map_name_input, "' != '' else ",
        "('sim_' + '", world, "'.rsplit('/', 1)[-1].replace('.world','') ",
        "if '", mode, "' == 'sim' else 'real_map')"
    ])
    # Save outside install/ so maps survive `colcon build`. Conventional
    # ROS scratch location.
    maps_dir = os.path.expanduser('~/.ros/scout_maps')
    os.makedirs(maps_dir, exist_ok=True)
    map_base_path = PathJoinSubstitution([maps_dir, final_map_name])
    hazards_file = PathJoinSubstitution([maps_dir,
                                         PythonExpression(
                                             ["'", final_map_name, "' + '_hazards.json'"]
                                         )])

    nav_params = PathJoinSubstitution([pkg_scout, 'config', 'scout_params.yaml'])
    nav_params_sim = PathJoinSubstitution([pkg_scout, 'config', 'scout_params_sim.yaml'])
    # Relaxed-tolerance variant used while SLAM is active (auto_mapper phase);
    # swaps back to scout_params.yaml for mission/nav tasks that need precision.
    nav_params_mapping = PathJoinSubstitution(
        [pkg_scout, 'config', 'scout_params_mapping.yaml']
    )
    nav_params_mapping_sim = PathJoinSubstitution(
        [pkg_scout, 'config', 'scout_params_mapping_sim.yaml']
    )
    slam_params = PathJoinSubstitution([pkg_scout, 'config', 'slam_params.yaml'])
    slam_params_sim = PathJoinSubstitution([pkg_scout, 'config', 'slam_params_sim.yaml'])
    explore_params = PathJoinSubstitution([pkg_scout, 'config', 'explore_params.yaml'])
    hazard_params = PathJoinSubstitution([pkg_scout, 'config', 'hazard_params.yaml'])
    waypoints_file = PathJoinSubstitution([pkg_scout, 'config', 'waypoints.txt'])

    rviz_config = os.path.join(pkg_tb3_nav, 'rviz', 'tb3_navigation2.rviz')

    # Make sure our Gazebo models (cubes, dock, UR7 placeholder) resolve.
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    our_models = os.path.join(pkg_scout, 'models')
    combined_model_path = (
        our_models if not existing_model_path else f'{our_models}:{existing_model_path}'
    )
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=combined_model_path
    )

    # Default to burger_cam in sim unless the user already set something else.
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=os.environ.get('TURTLEBOT3_MODEL', 'burger_cam'),
    )

    # ---------- condition expressions (consumed by PythonExpression) ----------
    # Each of these is a list of substitutions that PythonExpression concatenates
    # into a single Python boolean expression. Combined via simple list addition
    # to build compound conditions (see _cond() calls below).
    sim_ex = ["'", mode, "' == 'sim'"]
    real_ex = ["'", mode, "' == 'real'"]
    map_task_ex = ["'", task, "' == 'map'"]
    mission_task_ex = ["'", task, "' == 'mission'"]
    scout_task_ex = ["'", task, "' == 'scout'"]
    auto_map_ex = ["'", auto_map, "' == 'true'"]

    # Does this task need a live SLAM session? (scout + map)
    needs_slam_ex = ["'", task, "' in ('map', 'scout')"]
    # Does this task load a saved map into Nav2? (nav + mission)
    needs_saved_map_ex = ["'", task, "' in ('nav', 'mission')"]
    # Does this task run perception? (scout - we also allow it in map for dev)
    needs_perception_ex = ["'", task, "' in ('scout', 'map')"]
    # Does this task run the mission_manager? (scout + mission + nav, but nav
    # skips if no hazards/waypoints; we still launch it so the /mission_status
    # topic is always available for the arm side to subscribe).
    needs_mission_ex = ["'", task, "' in ('scout', 'nav', 'mission')"]

    # ============================================================ 1. SIM LAYER
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_scout, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items(),
        condition=_cond(sim_ex),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_scout, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'use_sim_time': 'true',
        }.items(),
        condition=_cond(sim_ex),
    )

    # ============================================================ 2. RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=_cond(needs_slam_ex),  # loaded-map tasks already launch their own
    )

    # ============================================================ 3. SLAM
    scan_resampler = Node(
        package='scout_system',
        executable='scan_resampler',
        name='scan_resampler',
        output='screen',
        remappings=[('/scan', '/scout/scan')],
        condition=_cond(real_ex + [' and '] + needs_slam_ex),
    )

    slam_sim = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_sim, {'use_sim_time': True}],
        condition=_cond(sim_ex + [' and '] + needs_slam_ex),
    )

    slam_real = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}],
        remappings=[('scan', 'scan_filtered')],
        condition=_cond(real_ex + [' and '] + needs_slam_ex),
    )

    # Nav2 (mapping config: no AMCL, map comes from SLAM).
    # Uses the relaxed-tolerance params so the auto_mapper's frontier goals
    # don't waste seconds inching to exact poses.
    nav2_mapping = GroupAction(
        actions=[
            SetRemap(
                src='/cmd_vel', dst='/scout/cmd_vel',
                condition=_cond(real_ex),
            ),
            SetRemap(
                src='/odom', dst='/scout/odom',
                condition=_cond(real_ex),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': PythonExpression(sim_ex),
                    'params_file': PythonExpression(
                        ["'", nav_params_mapping_sim, "' if '", mode, "' == 'sim' else '", nav_params_mapping, "'"]
                    ),
                    'map_subscribe_transient_local': 'true',
                }.items(),
            ),
        ],
        condition=_cond(needs_slam_ex
                        + [' and not ('] + map_task_ex + [' and not '] + auto_map_ex + [')']),
    )

    # ============================================================ 4. AUTO MAPPER
    # Our custom bounded-frontier explorer (replaces explore_lite).
    auto_mapper = Node(
        package='scout_system',
        executable='auto_mapper',
        name='auto_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(sim_ex),
            'map_path': PathJoinSubstitution([map_base_path]),
            'robot_frame': PythonExpression(
                ["'base_link' if '", mode, "' == 'sim' else 'scout/base_link'"]
            ),
            'completion_timeout': 10.0,
            'save_interval': 0.0,
            # Default in code is 3 -- way too tight given Nav2/SLAM warm-up.
            # First few ticks often fail (Nav2 not ready) which blacklists a
            # target, then there are no frontiers because /map has barely any
            # free cells yet, and the run "completes" before it ever started.
            # 60 ticks at tick_period=1.0s = 60 s grace, which comfortably
            # covers warm-up. Bring it back down once exploration starts
            # converging if you want a snappier finish.
            'max_consecutive_empty': 10,
            # scout flow needs to keep running to transition into mission;
            # pure 'map' mode can shut down at the end if you want, but we
            # leave it up so Ctrl-C always belongs to you.
            'shutdown_on_complete': False,
        }],
        condition=_cond(needs_slam_ex + [' and ('] + scout_task_ex
                        + [" or ("] + map_task_ex + [' and '] + auto_map_ex + [')'] + [')']),
    )

    manual_mapper = Node(
        package='scout_system',
        executable='manual_mapper',
        name='manual_mapper',
        output='screen',
        prefix='xterm -e',
        parameters=[{'map_path': PathJoinSubstitution([map_base_path])}],
        condition=_cond(map_task_ex + [' and not '] + auto_map_ex),
    )

    # ============================================================ 5. PERCEPTION
    hazard_detector = Node(
        package='scout_system',
        executable='hazard_detector',
        name='hazard_detector',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(sim_ex),
            'params_file': hazard_params,
            'image_topic': PythonExpression(
                ["'/camera/image_raw' if '", mode, "' == 'sim' else '/scout/image_raw'"]
            ),
            'camera_info_topic': PythonExpression(
                ["'/camera/camera_info' if '", mode, "' == 'sim' else '/scout/camera_info'"]
            ),
            'map_frame': 'map',
            'lidar_frame': PythonExpression(
                ["'base_scan' if '", mode, "' == 'sim' else 'scout/base_scan'"]
            ),
            # All other tuning (depth-jump, cluster width, bearing tol, etc.)
            # lives in hazard_detector.py defaults; override here if needed.
            # 3.1 m collapses the lidar's 3.5 m hardware limit and the
            # 0.4 m wall-ghost trust margin into a single trust threshold.
            'max_range_m': 3.1,
            # Require >= 2 lidar rays per cluster -- single-ray clusters
            # are usually noise. Cones beyond ~3 m subtend 1 ray only,
            # so this also caps effective range; lower to 1 if you need
            # those long-range hits and accept more noise.
            'cluster_min_rays': 3,
            # Cone is 14 cm wide. 0.22 m gives ~5 cm slack for ray noise
            # while still cleanly rejecting the 0.5 m pillar.
            'cluster_max_width_m': 0.22,
        }],
        condition=_cond(needs_perception_ex),
    )

    hazard_tracker = Node(
        package='scout_system',
        executable='hazard_tracker',
        name='hazard_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(sim_ex),
            # Cones in the arena are >= 3.2 m apart, so 1.0 m is a
            # comfortable merge ceiling -- catches any noisy fusion
            # that lands a meter off without ever conflating two
            # real cones.
            'merge_radius_m': 1.0,
            'min_observations': 3,
            # We have exactly 3 cones, one per color. Hard cap stops
            # phantom tracks: any same-colour observation gets folded
            # into the existing track, even if it's outside merge_radius.
            'max_tracks_per_color': 1,
            'hazards_file': PathJoinSubstitution([hazards_file]),
            'republish_period_s': 2.0,
        }],
        condition=_cond(needs_perception_ex + [' or '] + mission_task_ex),
    )

    # ============================================================ 6. NAV (saved-map tasks)
    nav2_saved = GroupAction(
        actions=[
            SetRemap(
                src='/cmd_vel', dst='/scout/cmd_vel',
                condition=_cond(real_ex),
            ),
            SetRemap(
                src='/odom', dst='/scout/odom',
                condition=_cond(real_ex),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_tb3_nav, 'launch', 'navigation2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': PythonExpression(sim_ex),
                    'map': [map_base_path, '.yaml'],
                    'params_file': PythonExpression(
                        ["'", nav_params_sim, "' if '", mode, "' == 'sim' else '", nav_params, "'"]
                    ),
                    'use_rviz': 'True',
                }.items(),
            ),
        ],
        condition=_cond(needs_saved_map_ex),
    )

    # ============================================================ 7. MISSION + UR7 STUB
    ur7_stub = Node(
        package='scout_system',
        executable='ur7_stub',
        name='ur7_stub',
        output='screen',
        parameters=[{'prep_seconds': 2.0, 'always_accept': True}],
        condition=_cond(needs_mission_ex),
    )

    mission_manager = Node(
        package='scout_system',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
            'standoff_distance': 0.6,
            'hazards_wait_timeout': 3.0,
            'request_package_timeout': 15.0,
        }],
        # In 'scout' task mode we launch it immediately; it will internally
        # wait for Nav2 + localization + the first confirmed hazard.
        condition=_cond(needs_mission_ex),
    )

    return LaunchDescription([
        mode_arg, task_arg, auto_map_arg, world_arg, map_name_arg,
        x_pose_arg, y_pose_arg,
        set_model_path, set_tb3_model,
        gazebo, spawn_robot,
        rviz,
        scan_resampler, slam_sim, slam_real, nav2_mapping,
        auto_mapper, manual_mapper,
        hazard_detector, hazard_tracker,
        nav2_saved,
        ur7_stub, mission_manager,
    ])
