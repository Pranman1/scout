# Namespace Handling in Scout System

## Overview

The scout system is designed to work in both simulation and real robot environments. The key difference is namespace handling for robot frames and topics.

## Namespacing Convention

### Simulation Mode (`mode:=sim`)
- **Robot frames**: No namespace prefix
  - `base_link`, `base_footprint`, `odom`, `base_scan`
- **Robot topics**: No namespace prefix
  - `/scan`, `/camera/image_raw`, `/camera/camera_info`, `/cmd_vel`
- **World frames**: No namespace
  - `map`
- **Application topics**: Global (no namespace)
  - `/hazards/raw`, `/hazards/confirmed`, `/scout/mapping_complete`, etc.

### Real Robot Mode (`mode:=real`)
- **Robot frames**: Prefixed with `scout/`
  - `scout/base_link`, `scout/base_footprint`, `scout/odom`, `scout/base_scan`
- **Robot topics**: Prefixed with `/scout/`
  - `/scout/scan`, `/scout/camera/image_raw`, `/scout/camera/camera_info`, `/scout/cmd_vel`
- **World frames**: No namespace (global)
  - `map`
- **Application topics**: Global (no namespace)
  - `/hazards/raw`, `/hazards/confirmed`, `/scout/mapping_complete`, etc.

## Implementation

### Node Configuration

Each node has a `robot_namespace` parameter that specifies the namespace prefix for robot frames:
- In sim: `robot_namespace: ''` (empty string)
- In real: `robot_namespace: 'scout'`

Each node implements a helper method `_add_namespace(frame_id)` that:
1. Returns `frame_id` unchanged if `robot_namespace` is empty or `frame_id` starts with '/'
2. Returns `{robot_namespace}/{frame_id}` otherwise

### Modified Files

#### Python Nodes
All nodes have been updated with namespace handling:
- `hazard_detector.py`: Namespaces `lidar_frame` (e.g., `base_scan` → `scout/base_scan`)
- `hazard_tracker.py`: Handles `map_frame` (always `map`, not namespaced)
- `auto_mapper.py`: Namespaces `robot_frame` (e.g., `base_link` → `scout/base_link`)
- `mission_manager.py`: Handles `map_frame` (always `map`, not namespaced)

#### Launch Files
`system.launch.py`:
- Sets `robot_namespace` parameter based on `mode` argument
- Uses different Nav2 parameter files for sim vs real
- Sets topic names with appropriate prefixes for real mode

#### Nav2 Parameters
Created separate parameter files for real mode:
- `scout_params.yaml` (sim) → `scout_params_real.yaml` (real)
- `scout_params_mapping.yaml` (sim) → `scout_params_mapping_real.yaml` (real)

The `_real.yaml` files have all robot frame references updated:
- `base_frame_id: "base_footprint"` → `base_frame_id: "scout/base_footprint"`
- `robot_base_frame: base_link` → `robot_base_frame: scout/base_link`
- `odom_frame_id: "odom"` → `odom_frame_id: "scout/odom"`
- `global_frame: odom` → `global_frame: scout/odom` (for local costmap)
- `local_frame: odom` → `local_frame: scout/odom` (for behavior server)

Note: `map` frame is NEVER namespaced - it remains `map` in both sim and real.

#### SLAM Parameters
`slam_params.yaml`:
- Launch file overrides `odom_frame` and `base_frame` for real mode with `scout/` prefix

## Usage

### Simulation
```bash
ros2 launch scout_system system.launch.py mode:=sim task:=scout
```

### Real Robot
```bash
# On robot Pi (separate terminal):
ros2 launch turtlebot3_bringup robot.launch.py namespace:=scout

# On laptop:
source ~/turtle_test/install/setup.bash  # TurtleBot underlay
source ~/scout_ws/install/setup.bash     # Scout overlay
ros2 launch scout_system system.launch.py mode:=real task:=scout
```

## Troubleshooting

### TF Frame Issues
If you see TF errors about missing frames:
1. Check that the robot is launched with `namespace:=scout` in real mode
2. Verify that SLAM/Nav2 are using the correct parameter files (_real.yaml for real mode)
3. Run `ros2 run tf2_tools view_frames` to visualize the TF tree

### Topic Issues
If nodes can't find topics:
1. Check that topics have the correct namespace prefix in real mode
2. Run `ros2 topic list` to see all available topics
3. Verify the `robot_namespace` parameter is set correctly in the launch file

### Common Issues
- **"Map → robot transform not available"**: Usually means SLAM or Nav2 is looking for the wrong robot frame name
- **"No scan data received"**: Check that scan_topic matches the actual topic name (with namespace in real mode)
- **"No camera data"**: Verify camera topics have correct namespace prefix in real mode

## Design Rationale

### Why namespace robot frames but not world frames?
- Robot frames (`base_link`, `odom`, etc.) need namespacing to avoid conflicts when multiple robots are present
- World frames (`map`) are global and shared across all robots
- This follows ROS2 multi-robot conventions

### Why separate parameter files for Nav2?
- Nav2 has deeply nested parameters that are difficult to override at launch time
- Separate files are more maintainable and easier to validate
- Allows version control of both sim and real configurations

### Why not launch all nodes in a namespace?
- Application-level topics (`/hazards/*`, `/scout/*`) need to be accessible globally
- UR7 arm and mission manager need to communicate across namespaces
- Explicit frame name handling gives more control and is easier to debug

## Testing

To verify namespace handling is working:

```bash
# Check TF frames
ros2 run tf2_tools view_frames
# Should show: map → scout/odom → scout/base_footprint → scout/base_link → scout/base_scan (real mode)
# Should show: map → odom → base_footprint → base_link → base_scan (sim mode)

# Check topics
ros2 topic list | grep scout
# Real mode should show: /scout/scan, /scout/camera/*, /scout/cmd_vel, etc.
# Sim mode should show: /scan, /camera/*, /cmd_vel (no /scout prefix)

# Check hazard detection is working
ros2 topic echo /hazards/raw
# Should see hazard messages in both sim and real mode
```
