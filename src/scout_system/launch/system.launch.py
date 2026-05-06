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
The real TurtleBot is brought up in the ``scout`` ROS namespace (via
``ros2 launch turtlebot3_bringup robot.launch.py namespace:=scout``)
because the same machine also hosts a UR7 arm whose URDF declares a
``base_link`` -- without the namespace the two TF trees collide.

So in real mode every robot-data topic (cmd_vel, scan, odom,
image_raw, camera_info) and every robot TF frame (base_link,
base_footprint, base_scan, odom) gets a ``scout/`` prefix. The ``map``
frame stays global, and the scout-system coordination topics
(``/scout/mapping_complete``, ``/scout/package_request`` ...) are
already absolute under ``/scout/...`` so they're identical in both
modes. This launch file folds all of that in via parameter / remap
choices keyed on the ``mode`` argument; the node code itself only
needs to be parameter-driven (which it already is).

The scout_ws is intended to overlay on an underlay workspace that
provides the stock turtlebot3 bringup (typically ~/turtle_test). Source
both before launching::

    source ~/turtle_test/install/setup.bash
    source ~/scout_ws/install/setup.bash

In real mode this launch file does NOT bring up the robot base -- run
``ros2 launch turtlebot3_bringup robot.launch.py namespace:=scout`` on
the Pi yourself. Only nav-stack / perception / mission pieces launch
here.
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


def _ternary(mode_lc, real_value, sim_value):
    """Substitution that resolves to ``real_value`` when mode==real, else ``sim_value``.

    Both values are plain strings. Keeps every "X in real / Y in sim"
    decision in this file readable and one-line.
    """
    return PythonExpression([
        "'", real_value, "' if '", mode_lc, "' == 'real' else '", sim_value, "'"
    ])


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

    # ---------- per-mode params files / topic / frame names ----------
    # Sim uses the original yaml; real uses the namespaced (_real) yaml
    # with scout/base_link, /scout/scan etc. baked in. Picking via
    # PythonExpression keeps this single launch file driving both.
    nav_params_sim = os.path.join(pkg_scout, 'config', 'scout_params.yaml')
    nav_params_real = os.path.join(pkg_scout, 'config', 'scout_params_real.yaml')
    nav_params_mapping_sim = os.path.join(
        pkg_scout, 'config', 'scout_params_mapping.yaml'
    )
    nav_params_mapping_real = os.path.join(
        pkg_scout, 'config', 'scout_params_mapping_real.yaml'
    )
    nav_params = _ternary(mode, nav_params_real, nav_params_sim)
    nav_params_mapping = _ternary(mode, nav_params_mapping_real, nav_params_mapping_sim)

    slam_params = PathJoinSubstitution([pkg_scout, 'config', 'slam_params.yaml'])
    explore_params = PathJoinSubstitution([pkg_scout, 'config', 'explore_params.yaml'])
    hazard_params = PathJoinSubstitution([pkg_scout, 'config', 'hazard_params.yaml'])
    waypoints_file = PathJoinSubstitution([pkg_scout, 'config', 'waypoints.txt'])

    rviz_config = os.path.join(pkg_tb3_nav, 'rviz', 'tb3_navigation2.rviz')

    # Frame IDs. The map frame is global in both modes; everything that
    # rides on the robot's TF tree gets prefixed in real mode because the
    # turtlebot3 bringup is launched with namespace=scout.
    base_link_frame = _ternary(mode, 'scout/base_link', 'base_link')
    base_footprint_frame = _ternary(mode, 'scout/base_footprint', 'base_footprint')
    base_scan_frame = _ternary(mode, 'scout/base_scan', 'base_scan')
    odom_frame = _ternary(mode, 'scout/odom', 'odom')

    # Robot-data topic names. Internal scout coordination topics
    # (/scout/mapping_complete etc.) are absolute and identical across
    # modes -- no need to remap or pass them as parameters.
    #
    # Camera asymmetry note: in sim the burger_cam Gazebo plugin
    # publishes under /camera/* and shares the namespaced robot. On the
    # real robot the Logitech webcam is launched as a separate
    # standalone node at root namespace (it's *not* part of the
    # turtlebot3_bringup namespace push), so its topics live at
    # /image_raw and /camera_info -- NOT /scout/image_raw. The
    # RealSense at /camera/camera/* is the UR7's and we don't touch it.
    cmd_vel_topic = _ternary(mode, '/scout/cmd_vel', '/cmd_vel')
    scan_topic = _ternary(mode, '/scout/scan', '/scan')
    odom_topic = _ternary(mode, '/scout/odom', '/odom')
    image_topic = _ternary(mode, '/image_raw', '/camera/image_raw')
    camera_info_topic = _ternary(
        mode, '/camera_info', '/camera/camera_info'
    )

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
    # In real mode the LDS-02 emits a variable ray count (250-256), which
    # confuses slam_toolbox's scan-matching; scan_resampler interpolates
    # back up to a fixed 360 rays on /scan_filtered.
    scan_resampler = Node(
        package='scout_system',
        executable='scan_resampler',
        name='scan_resampler',
        output='screen',
        # Subscribe to the namespaced raw scan from the robot, publish
        # the resampled stream on a global /scan_filtered so SLAM and
        # any other consumer at root namespace can pick it up.
        remappings=[
            ('scan', '/scout/scan'),
            ('scan_filtered', '/scan_filtered'),
        ],
        condition=_cond(real_ex + [' and '] + needs_slam_ex),
    )

    slam_sim = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
        condition=_cond(sim_ex + [' and '] + needs_slam_ex),
    )

    slam_real = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        # Override base/odom frames so SLAM publishes the
        # map -> scout/odom transform that the namespaced TF tree
        # expects. /map stays a global topic (slam_toolbox publishes it
        # as absolute), so auto_mapper / Nav2 don't care about this
        # rename. scan_topic is read directly by slam_toolbox to build
        # its subscriber, so we override it (a topic remap on 'scan'
        # would not take effect because the create_subscription call
        # uses whatever scan_topic resolves to as the absolute name).
        parameters=[
            slam_params,
            {
                'use_sim_time': False,
                'base_frame': base_footprint_frame,
                'odom_frame': odom_frame,
                'scan_topic': '/scan_filtered',
            },
        ],
        condition=_cond(real_ex + [' and '] + needs_slam_ex),
    )

    # Nav2 (mapping config: no AMCL, map comes from SLAM).
    # Uses the relaxed-tolerance params so the auto_mapper's frontier goals
    # don't waste seconds inching to exact poses.
    #
    # In real mode we wrap the include in a GroupAction with SetRemap so
    # that every Nav2 node inside it picks up the namespaced robot
    # topics. /tf and /tf_static are global out of the box (turtlebot3
    # bringup remaps them), so we don't have to touch those.
    nav2_mapping_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav_params_mapping,
            'map_subscribe_transient_local': 'true',
        }.items(),
        condition=_cond(sim_ex + [' and ('] + needs_slam_ex
                        + [' and not ('] + map_task_ex
                        + [' and not '] + auto_map_ex + [')'] + [')']),
    )
    # Real-mode wrapper: SetRemap propagates relative cmd_vel / scan / odom
    # to /scout/* across every node inside the include, including the ones
    # created by nav2_bringup internally.
    nav2_mapping_real = GroupAction(
        actions=[
            SetRemap('cmd_vel', '/scout/cmd_vel'),
            SetRemap('cmd_vel_smoothed', '/scout/cmd_vel'),
            SetRemap('scan', '/scout/scan'),
            SetRemap('odom', '/scout/odom'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav_params_mapping,
                    'map_subscribe_transient_local': 'true',
                }.items(),
            ),
        ],
        condition=_cond(real_ex + [' and ('] + needs_slam_ex
                        + [' and not ('] + map_task_ex
                        + [' and not '] + auto_map_ex + [')'] + [')']),
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
            'completion_timeout': 10.0,
            'save_interval': 0.0,
            'map_frame': 'map',
            'robot_frame': base_link_frame,
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

    # Manual WASD mapper. Real-mode remap so its 'cmd_vel' lands on the
    # /scout/cmd_vel topic the namespaced base subscribes to.
    manual_mapper = Node(
        package='scout_system',
        executable='manual_mapper',
        name='manual_mapper',
        output='screen',
        prefix='xterm -e',
        parameters=[{'map_path': PathJoinSubstitution([map_base_path])}],
        remappings=[('cmd_vel', cmd_vel_topic)],
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
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'scan_topic': scan_topic,
            'map_frame': 'map',
            'lidar_frame': base_scan_frame,
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
            'map_frame': 'map',
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
    # Same SetRemap trick as the mapping-phase Nav2 above so the saved-map
    # AMCL+Nav2 stack drives the namespaced base in real mode.
    nav2_saved_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': [map_base_path, '.yaml'],
            'params_file': nav_params,
            'use_rviz': 'True',
        }.items(),
        condition=_cond(sim_ex + [' and '] + needs_saved_map_ex),
    )
    nav2_saved_real = GroupAction(
        actions=[
            SetRemap('cmd_vel', '/scout/cmd_vel'),
            SetRemap('cmd_vel_smoothed', '/scout/cmd_vel'),
            SetRemap('scan', '/scout/scan'),
            SetRemap('odom', '/scout/odom'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_tb3_nav, 'launch', 'navigation2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'map': [map_base_path, '.yaml'],
                    'params_file': nav_params,
                    'use_rviz': 'True',
                }.items(),
            ),
        ],
        condition=_cond(real_ex + [' and '] + needs_saved_map_ex),
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
            'use_sim_time': PythonExpression(sim_ex),
            'map_frame': 'map',
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
        scan_resampler, slam_sim, slam_real,
        nav2_mapping_sim, nav2_mapping_real,
        auto_mapper, manual_mapper,
        hazard_detector, hazard_tracker,
        nav2_saved_sim, nav2_saved_real,
        ur7_stub, mission_manager,
    ])
