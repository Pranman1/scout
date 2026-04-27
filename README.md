# Scout System — EECS 106A Final Project

A TurtleBot3-based "remote drone" that explores an environment, detects
colored-cube hazards with its onboard camera, and coordinates with a UR7
arm (separate package) to deliver a response package to each hazard.

This workspace contains **only the scout side** of the system. It is
designed to be overlaid on top of a standard TurtleBot3 workspace (the
`turtle_test` ws next door) which provides the stock robot model, Gazebo
sim, and laser drivers.

---

## Table of Contents
1. [What's in the box](#whats-in-the-box)
2. [Prerequisites and Underlay Setup](#prerequisites-and-underlay-setup)
3. [Build](#build)
4. [Launch Commands](#launch-commands)
5. [How the Pipeline Fits Together](#how-the-pipeline-fits-together)
6. [Configuration Cheat Sheet](#configuration-cheat-sheet)
7. [Swapping in YOLO Later](#swapping-in-yolo-later)
8. [Wiring In the Real UR7](#wiring-in-the-real-ur7)
9. [Troubleshooting](#troubleshooting)
10. [File Layout](#file-layout)

---

## What's in the box

| Package        | Language    | Role                                                     |
|----------------|-------------|----------------------------------------------------------|
| `scout_msgs`   | ament_cmake | Custom `Hazard.msg`, `MissionStatus.msg`, `RequestPackage.srv` |
| `scout_system` | ament_python | Launch files, nodes, Gazebo world + models, config        |

**Nodes in `scout_system`:**

| Node              | Purpose                                                       |
|-------------------|---------------------------------------------------------------|
| `hazard_detector` | HSV (or pluggable) color-blob detection on camera frames, projects into map frame |
| `hazard_tracker`  | Deduplicates raw detections into stable tracks; latches the confirmed list |
| `mission_manager` | Waits for hazards, drives the robot to each, calls `/request_package` |
| `auto_mapper`     | Watches `explore_lite` frontiers, saves the map when done, signals completion |
| `manual_mapper`   | WASD teleop with 'p' to save (opens its own xterm)            |
| `scan_resampler`  | Real-robot LDS-02 fix: pins scan count to 360 via interpolation |
| `ur7_stub`        | Stand-in service server for `/request_package` until the real UR7 is ready |

**Gazebo assets:**

* `worlds/scout_arena.world` — 8 m × 8 m walled room, inner pillar, 3 hazard cubes, dock pad, UR7 placeholder
* `models/hazard_cube_{red,yellow,blue}` — semantic: fire / chemical / medical
* `models/scout_dock` — flat 0.6 × 0.6 m pad at the robot home
* `models/ur7_placeholder` — non-functional grey box where the arm will live

---

## Prerequisites and Underlay Setup

You need:

1. **Ubuntu 22.04 + ROS 2 Humble** (same platform as the TurtleBot3 stack).
2. **`turtle_test` workspace already built** — it provides `turtlebot3_gazebo`,
   `turtlebot3_navigation2`, `explore_lite`, etc. If you haven't built it yet:

   ```bash
   cd ~/turtle_test
   colcon build --symlink-install
   ```

3. **Extra apt deps we need beyond what turtle_test pulls in:**

   ```bash
   sudo apt install -y \
     ros-humble-cv-bridge \
     ros-humble-image-transport \
     ros-humble-vision-opencv \
     ros-humble-nav2-simple-commander \
     python3-opencv \
     xterm
   ```

4. **TURTLEBOT3_MODEL** must include a camera. The scout defaults to the stock
   `turtlebot3_burger_cam` model (Burger base + front camera with
   `/camera/image_raw` + `/camera/camera_info`). The launch file sets this env
   var for you if it's unset, but adding it to `~/.bashrc` is harmless:

   ```bash
   echo 'export TURTLEBOT3_MODEL=burger_cam' >> ~/.bashrc
   ```

---

## Build

Always build the underlay first, overlay second:

```bash
# 1) underlay (once, or whenever you update third-party packages)
cd ~/turtle_test
colcon build --symlink-install

# 2) overlay (this repo)
source ~/turtle_test/install/setup.bash
cd ~/scout_ws
colcon build --symlink-install
```

From then on, every new terminal you use for scout work needs:

```bash
source ~/turtle_test/install/setup.bash
source ~/scout_ws/install/setup.bash
```

---

## Launch Commands

Everything runs through a single launch file with four task modes.

### Full demo: map + detect + run mission (recommended)

```bash
ros2 launch scout_system system.launch.py mode:=sim task:=scout
```

The robot starts at the dock, `explore_lite` drives it around, the
`hazard_detector` watches the camera feed, and the `hazard_tracker`
promotes stable detections to confirmed hazards. Once exploration
completes, the saved map + a snapshot of confirmed hazards is written
to `install/scout_system/share/scout_system/maps/`, and the
`mission_manager` starts visiting them in order, calling the (stubbed)
UR7 service at each one.

### Mapping only

```bash
# Auto: explore_lite drives
ros2 launch scout_system system.launch.py mode:=sim task:=map auto_map:=true

# Manual: xterm pops up, WASD drives, 'p' to save
ros2 launch scout_system system.launch.py mode:=sim task:=map auto_map:=false
```

### Navigate a saved map (no mission)

```bash
ros2 launch scout_system system.launch.py mode:=sim task:=nav map_name:=sim_scout_arena
```

You'll need to set the initial pose in RViz ("2D Pose Estimate").

### Replay the mission on a saved map + saved hazards

```bash
ros2 launch scout_system system.launch.py mode:=sim task:=mission map_name:=sim_scout_arena
```

The tracker reads the previously dumped `*_hazards.json` and the
mission_manager visits each.

### Real robot

On the Pi (after `ssh`):

```bash
export TURTLEBOT3_MODEL=burger   # the real Burger; camera is Logitech via USB
ros2 launch turtlebot3_bringup robot.launch.py
# (bring up the Logitech via v4l2_camera or usb_cam, remapping to /camera/*)
```

On the laptop:

```bash
ros2 launch scout_system system.launch.py mode:=real task:=scout
```

---

## How the Pipeline Fits Together

```
                         ┌──────────────────────────────────┐
                         │         scout_system             │
                         │        system.launch.py          │
                         │                                  │
              ┌──────────┤  mode / task / auto_map / world  │
              │          └──────────────────────────────────┘
              │
   ┌──────────┴──────────────────────────────┐
   │                                         │
 SIM LAYER (mode=sim)                    REAL LAYER (mode=real)
 - gazebo.launch.py                      - (robot bringup done externally)
 - spawn_robot.launch.py                 - scan_resampler: /scan -> /scan_filtered
   (burger_cam at dock pose)             - logitech usb/v4l2 -> /camera/*

         ┌──────────────┴───────────────┐
         ▼                              ▼
   ┌─────────────┐   task=scout/map     ┌────────────────────┐
   │ slam_toolbox│◄────── /scan ────────┤  hazard_detector   │
   │             │                      │  (HSV backend)     │
   │ Nav2 stack  │                      │  /camera/image_raw │
   └─────┬───────┘                      │  /camera/camera_info│
         │  /map                        └─────────┬──────────┘
         ▼                                        │ /hazards/raw
   ┌─────────────┐   task=scout/map               ▼
   │ explore_lite│                        ┌───────────────┐
   └─────┬───────┘                        │ hazard_tracker│
         │ /explore/frontiers             └──────┬────────┘
         ▼                                       │ /hazards/confirmed (latched)
   ┌─────────────┐                               │
   │ auto_mapper │──────► /scout/mapping_complete│
   │  saves map  │                               ▼
   └─────────────┘                       ┌────────────────┐
                                         │ mission_manager│────► /mission_status
                                         │                │────► /request_package
                                         └────────┬───────┘
                                                  │
                                          ┌───────▼────────┐
                                          │   ur7_stub     │ (to be replaced by
                                          │ fake RequestPkg│  real UR7 driver)
                                          └────────────────┘
```

Key topics / services you can inspect from `ros2 topic echo`:

| Topic / Service                | Type                              | Notes                          |
|-------------------------------|-----------------------------------|--------------------------------|
| `/hazards/raw`                 | `scout_msgs/Hazard`               | Stream, per detection frame    |
| `/hazards/confirmed`           | `scout_msgs/Hazard`               | Latched, deduplicated          |
| `/hazards/confirmed/markers`   | `visualization_msgs/MarkerArray`  | For RViz                       |
| `/scout/mapping_complete`      | `std_msgs/Bool`                   | Latched; flips true when done  |
| `/mission_status`              | `scout_msgs/MissionStatus`        | Human-readable status feed     |
| `/request_package` (service)   | `scout_msgs/RequestPackage`       | Scout → arm handshake          |

---

## Configuration Cheat Sheet

Nearly every tunable lives in `src/scout_system/config/`:

| File                 | Controls                                                 |
|----------------------|----------------------------------------------------------|
| `scout_params.yaml`  | Nav2 (AMCL, costmaps, controller, planner, behaviors)    |
| `slam_params.yaml`   | SLAM Toolbox                                             |
| `explore_params.yaml`| explore_lite                                             |
| `hazard_params.yaml` | HSV ranges, color → category mapping, tracker knobs      |
| `waypoints.txt`      | Fallback targets if no hazards have been recorded yet    |

Most useful knobs during dev:

* **Too many false positives?** Raise `detector.min_area_px` or tighten HSV
  lows (`S` / `V` mins).
* **Too few detections?** Drop `min_area_px`, broaden HSV.
* **Hazards merging into one track?** Decrease `tracker.merge_radius_m`.
* **Robot stops too far from hazard?** Decrease
  `mission_manager.standoff_distance` (launch-file param).

---

## Swapping in YOLO Later

The detector is behind a `Detector` interface (`scout_system/detectors/base.py`).
To use YOLO:

1. Fill in `scout_system/detectors/yolo.py` (load weights in `__init__`,
   implement `detect(image_bgr)` returning `List[Detection]`).
2. Set `detector.backend: yolo` in `hazard_params.yaml` and fill in the
   `yolo:` block.
3. Rebuild `scout_system`. No other code changes.

You can also run both in parallel later: subclass `Detector` into a
`CompositeDetector` that fuses results from HSV + YOLO (e.g. HSV for
fast triage, YOLO only on the HSV bounding boxes).

---

## Wiring In the Real UR7

1. Your UR7 package must expose a service server at `/request_package`
   implementing `scout_msgs/srv/RequestPackage`.
2. Disable `ur7_stub` (edit the launch file's condition, or just kill
   the node at runtime — first writer wins).
3. The `/mission_status` topic already tells the arm side exactly what
   phase the robot is in, which hazard is current, etc. No further
   plumbing needed on the scout side.

---

## Troubleshooting

### "Entity [burger_cam] already exists" / Gazebo won't start

Zombie sim processes. Kill everything:

```bash
killall -9 gzserver gzclient
pkill -9 -f ros
ros2 daemon stop && ros2 daemon start
```

### "Could not find turtlebot3_gazebo / explore_lite"

You forgot to source the underlay. Every shell needs:

```bash
source ~/turtle_test/install/setup.bash
source ~/scout_ws/install/setup.bash
```

### Hazard detector sees nothing

1. `ros2 topic hz /camera/image_raw` — is the camera publishing? If not,
   check `TURTLEBOT3_MODEL=burger_cam` is set in the shell you launched from.
2. `ros2 run rqt_image_view rqt_image_view /camera/image_raw` — what do the
   cubes actually look like? Gazebo lighting sometimes washes out hue.
3. Tune `hazard_params.yaml` while running — `cv2.cvtColor` and
   `cv2.inRange` are cheap; iterate fast.

### Hazards appear in weird places (like inside walls)

The v1 pose estimator assumes cubes sit on the floor. If your camera
pitch estimate is off, you'll get scale errors. Quick check:
`ros2 run tf2_tools view_frames.py` — camera_link should be at
(≈0.04, 0, 0.11) relative to base_link. If not, either the URDF is
wrong or you're in real mode without the right robot_state_publisher.

### Map doesn't save at end of exploration

`ros2 topic echo /explore/frontiers` — if it never publishes,
explore_lite isn't discovering frontiers (often because Nav2's global
costmap isn't populated yet). Drop `explore_params.yaml`'s
`min_frontier_size` further, or drive the robot manually for a few
seconds first.

### Real LDS-02: SLAM starts then freezes

That's exactly the variable-scan-count issue `scan_resampler` fixes.
Check `ros2 node list | grep scan_resampler` — it should be up in
`mode:=real`.

### Kill everything

```bash
pkill -9 -f ros2
pkill -9 -f rviz
pkill -9 -f gazebo
```

---

## File Layout

```
scout_ws/
├── README.md
├── .gitignore
└── src/
    ├── scout_msgs/                        # custom interfaces (ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── msg/
    │   │   ├── Hazard.msg
    │   │   └── MissionStatus.msg
    │   └── srv/
    │       └── RequestPackage.srv
    │
    └── scout_system/                      # everything else (ament_python)
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/scout_system
        ├── scout_system/
        │   ├── __init__.py
        │   ├── manual_mapper.py
        │   ├── auto_mapper.py
        │   ├── scan_resampler.py
        │   ├── mission_manager.py
        │   ├── hazard_detector.py
        │   ├── hazard_tracker.py
        │   ├── ur7_client_stub.py
        │   └── detectors/
        │       ├── __init__.py            # build_detector() factory
        │       ├── base.py                # Detector / Detection interface
        │       ├── hsv.py                 # HSV blob backend
        │       └── yolo.py                # stub, wire up weights later
        ├── launch/
        │   ├── system.launch.py           # main entry: mode + task
        │   ├── gazebo.launch.py           # gzserver + gzclient + world
        │   └── spawn_robot.launch.py      # turtlebot3 + robot_state_publisher
        ├── config/
        │   ├── scout_params.yaml          # Nav2
        │   ├── slam_params.yaml           # SLAM Toolbox
        │   ├── explore_params.yaml        # explore_lite
        │   ├── hazard_params.yaml         # detector + tracker
        │   └── waypoints.txt              # fallback targets
        ├── urdf/
        │   └── README_camera.md
        ├── worlds/
        │   └── scout_arena.world
        ├── models/
        │   ├── hazard_cube_red/
        │   ├── hazard_cube_yellow/
        │   ├── hazard_cube_blue/
        │   ├── scout_dock/
        │   └── ur7_placeholder/
        └── maps/                          # saved .yaml/.pgm + *_hazards.json land here
```

---

## License

MIT.
