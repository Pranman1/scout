# Camera setup notes

For **simulation** we reuse the stock `turtlebot3_burger_cam` model from
the turtlebot3_simulations package in `~/turtle_test`. Make sure this is
set before launching sim:

```bash
export TURTLEBOT3_MODEL=burger_cam
```

The stock model already publishes:
- `/camera/image_raw`   (sensor_msgs/Image)
- `/camera/camera_info` (sensor_msgs/CameraInfo)

...with `camera_link` as the frame. The scout nodes use those topic
names by default; see `config/hazard_params.yaml`.

For the **real robot**, the Logitech C920 publishes via `v4l2_camera_node`
(or `usb_cam`, depending on your bringup). In `mode:=real` the launch
file wires that up and remaps to the same `/camera/*` topics so no other
code has to care which world we're in.

If you ever want a custom URDF (e.g. different camera mount height /
different intrinsics), drop a `turtlebot3_burger_cam.urdf.xacro` in this
directory and point the launch file at it instead of the stock model.
