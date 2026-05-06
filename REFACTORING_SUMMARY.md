# Scout System Namespace Refactoring Summary

## Changes Made

This refactoring enables the scout system to work seamlessly in both simulation (no namespace) and real robot environments (with `scout/` namespace prefix for robot frames and topics).

### 1. Python Node Updates

#### hazard_detector.py
- Added `robot_namespace` parameter
- Added `_add_namespace()` helper method
- Updated frame handling:
  - `lidar_frame` is now namespaced using `_add_namespace()`
  - `map_frame` remains non-namespaced (global)
- Topics are set via launch file parameters

#### hazard_tracker.py  
- Added `robot_namespace` parameter
- Added `map_frame` parameter
- Added `_add_namespace()` helper method
- Updated `_track_to_hazard()` to use `self.map_frame`
- Updated `_make_marker()` to use `self.map_frame`

#### auto_mapper.py
- Added `robot_namespace` parameter
- Added `_add_namespace()` helper method
- Updated frame handling:
  - `robot_frame` is now namespaced using `_add_namespace()`
  - `map_frame` remains non-namespaced (global)

#### mission_manager.py
- Added `robot_namespace` parameter
- Added `map_frame` parameter
- Added `_add_namespace()` helper method
- Updated `PoseStamped` messages to use `self.map_frame` in:
  - `NAVIGATING_TO_HAZARD` state
  - `RETURNING_HOME` state

### 2. Launch File Updates (system.launch.py)

#### Parameter File Selection
- Added conditional selection of Nav2 param files based on mode:
  ```python
  'scout_params_real.yaml' if mode=='real' else 'scout_params.yaml'
  'scout_params_mapping_real.yaml' if mode=='real' else 'scout_params_mapping.yaml'
  ```

#### Node Parameters
Updated all perception and navigation nodes with:
- `robot_namespace`: Empty for sim, `'scout'` for real
- Topic names adjusted for real mode (e.g., `/scout/scan`, `/scout/camera/image_raw`)
- Frame names passed as base names (namespace added by nodes)

#### SLAM Configuration
- `slam_real` node now overrides `odom_frame` and `base_frame` with namespaced versions

### 3. New Configuration Files

Created real-mode Nav2 parameter files with namespaced robot frames:

#### scout_params_real.yaml
- `base_frame_id: "scout/base_footprint"`
- `odom_frame_id: "scout/odom"`  
- `robot_base_frame: scout/base_link`
- `global_frame: scout/odom` (local costmap)
- `local_frame: scout/odom` (behavior server)
- `global_frame: map` (bt_navigator, global costmap - not namespaced)

#### scout_params_mapping_real.yaml
- Same namespace updates as above
- Maintains relaxed tolerances for mapping phase

### 4. Documentation

#### NAMESPACE_HANDLING.md
Comprehensive guide covering:
- Namespacing conventions for sim vs real
- Implementation details
- Usage instructions
- Troubleshooting guide
- Design rationale
- Testing procedures

## Key Design Decisions

### 1. Frame Namespacing Convention
- **Robot frames**: Namespaced in real mode (`scout/base_link`, `scout/odom`, etc.)
- **World frames**: NEVER namespaced (`map` stays `map`)
- **Rationale**: Follows ROS2 multi-robot standards; map is global, robot frames are local

### 2. Node-Level Namespace Handling
- Each node has `robot_namespace` parameter and `_add_namespace()` helper
- **Rationale**: Centralized logic, easier to debug, explicit control

### 3. Separate Param Files for Nav2
- Created `*_real.yaml` variants instead of launch-time overrides
- **Rationale**: Nav2 has deeply nested params; separate files are more maintainable

### 4. Topic Namespacing
- Robot topics (scan, camera, cmd_vel): Namespaced in real mode
- Application topics (/hazards/*, /scout/*): Global (no namespace)
- **Rationale**: Application topics need cross-namespace access

## Testing Checklist

### Before Committing
- [ ] Verify all nodes compile: `colcon build`
- [ ] Check for Python lint errors: `flake8 src/scout_system/scout_system/*.py`
- [ ] Test simulation mode: `ros2 launch scout_system system.launch.py mode:=sim task:=scout`
- [ ] Verify TF tree in sim: `ros2 run tf2_tools view_frames`

### Real Robot Testing
- [ ] Launch robot with namespace: `ros2 launch turtlebot3_bringup robot.launch.py namespace:=scout`
- [ ] Launch scout system: `ros2 launch scout_system system.launch.py mode:=real task:=scout`
- [ ] Verify TF tree shows namespaced frames: `ros2 run tf2_tools view_frames`
- [ ] Check topics: `ros2 topic list | grep scout`
- [ ] Test hazard detection: `ros2 topic echo /hazards/raw`
- [ ] Test navigation: Verify auto_mapper and mission_manager work correctly
- [ ] Test full scout workflow: Mapping → detection → mission execution

## Files Modified

```
src/scout_system/
├── scout_system/
│   ├── hazard_detector.py
│   ├── hazard_tracker.py
│   ├── auto_mapper.py
│   └── mission_manager.py
├── launch/
│   └── system.launch.py
└── config/
    ├── scout_params_real.yaml (NEW)
    └── scout_params_mapping_real.yaml (NEW)

Documentation:
├── NAMESPACE_HANDLING.md (NEW)
└── REFACTORING_SUMMARY.md (NEW)
```

## Compatibility

### Backwards Compatibility
- **Simulation mode**: Fully backwards compatible, no changes to frame names or topics
- **Real mode**: Requires robot to be launched with `namespace:=scout`

### Forward Compatibility
- Design supports multiple robots (can add more namespaces in future)
- Separate param files allow independent tuning of sim vs real
- Helper methods make it easy to add new namespaced nodes

## Common Issues and Solutions

### Issue: "Transform timeout" errors in real mode
**Cause**: SLAM/Nav2 looking for wrong frame names  
**Solution**: Verify `*_real.yaml` param files are being used; check `robot_namespace` parameter

### Issue: No scan data in hazard detector
**Cause**: Incorrect scan topic name  
**Solution**: Verify `scan_topic` parameter matches actual topic (check with `ros2 topic list`)

### Issue: Hazards published in wrong frame
**Cause**: `map_frame` incorrectly namespaced  
**Solution**: Map frame should always be `'map'`, not `'scout/map'`

## Next Steps

1. **Test in simulation** to ensure backwards compatibility
2. **Test on real robot** with namespaced TurtleBot launch
3. **Update README.md** with namespace-aware usage instructions
4. **Create integration tests** for both sim and real modes
5. **Consider**: Abstract namespace handling into a base class for reuse

## Notes

- All changes maintain zero-diff behavior in simulation mode
- Real mode now properly handles UR7 + TurtleBot coexistence
- Frame name handling is explicit and debuggable
- No global namespace pollution in multi-robot scenarios
