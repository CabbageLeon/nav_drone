# RealSense Camera Setup

This project uses Intel RealSense ROS2 wrapper as the camera driver.

The currently connected RealSense device enumerates as `RealSense USB2`
with Product ID `0AD6` and exposes Infrared/Depth stream profiles, but no RGB
Camera profile. For this hardware, the SPF node subscribes to the verified
infrared image topic:

```bash
/camera/camera/infra1/image_rect_raw
```

The project config is already aligned in:

```bash
src/uav_controller/config/spf_params.yaml
```

## Sources

- Intel RealSense ROS2 wrapper documentation: https://dev.realsenseai.com/docs/ros2-wrapper
- RealSense ROS wrapper README: https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md
- ROS2 `sensor_msgs/Image` definition: https://docs.ros.org/en/humble/p/sensor_msgs/msg/Image.html

## Install

Use the ROS Humble Debian packages on Ubuntu 22.04:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-camera-msgs \
  ros-humble-realsense2-description
```

The `ros-humble-realsense2-camera` package depends on `ros-humble-librealsense2`,
so the RealSense SDK runtime is installed with the wrapper.

Verify the package is visible to ROS:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix realsense2_camera
```

If this prints `Package not found`, the wrapper is not installed in the active
ROS environment.

## Current Verified Device

The current machine has been verified with:

```bash
source /opt/ros/humble/setup.bash
rs-enumerate-devices
```

Observed device:

```text
Name              : RealSense USB2
Serial Number     : 819312071789
Product Id        : 0AD6
Product Line      : D400
Available streams : Infrared, Infrared 1, Infrared 2, Depth
Verified ROS topic: /camera/camera/infra1/image_rect_raw at 30 Hz
```

Because this device does not expose an RGB Camera stream, do not use
`/camera/camera/color/image_raw` on this machine unless a different D435 with
RGB support is connected.

## Connect Camera

1. Connect the RealSense camera.
2. Check that Linux sees the device:

```bash
lsusb | grep -i realsense
```

3. If multiple RealSense cameras are connected, get the serial number:

```bash
ros2 launch realsense2_camera rs_launch.py initial_reset:=true
```

Then set `serial_no:=_<serial>` in the launch command used for flight. The
leading `_` keeps the serial number parsed as a string by the ROS2 launch
argument parser.

## Start Infrared Stream For SPF

For the currently connected hardware, start Infrared 1 only. Disable RGB and
depth to reduce USB and CPU load:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=false \
  initial_reset:=true
```

The SPF image topic for this hardware is:

```bash
/camera/camera/infra1/image_rect_raw
```

This matches `spf_node.image_topic` in:

```bash
src/uav_controller/config/spf_params.yaml
```

## Start Project Nodes

In another terminal:

```bash
cd ~/UAV_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_bringup control_only.launch.py spf:=true
```

For simulation runs in this project, keep the DDS implementation explicit:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Verify

Check that the camera topic is publishing:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic info /camera/camera/infra1/image_rect_raw
```

Inspect one frame:

```bash
ros2 topic echo /camera/camera/infra1/image_rect_raw --once
```

Optional visual check:

```bash
rqt_image_view /camera/camera/infra1/image_rect_raw
```

## RGB D435 Alternative

If a different D435 exposes an RGB Camera stream, start the wrapper with color
enabled:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=false \
  initial_reset:=true
```

With the wrapper defaults, the RGB image topic is:

```bash
/camera/camera/color/image_raw
```

If this RGB stream is used, change `spf_node.image_topic` in
`src/uav_controller/config/spf_params.yaml` to:

```bash
/camera/camera/color/image_raw
```

## Old Project Topic

The old project image topic was:

```bash
/camera/down/image_raw
```

SPF no longer uses this topic. Prefer subscribing directly to the wrapper topic
that exists for the connected RealSense hardware. If a legacy node still
depends on the old name, add the remap in a project launch file and change only
that legacy node's config.

## Troubleshooting

- `Package not found`: install the Humble wrapper packages and source
  `/opt/ros/humble/setup.bash`.
- No `/camera/camera/infra1/image_rect_raw`: check USB connection, restart with
  `initial_reset:=true`, and confirm no other process is using the camera.
- No `/camera/camera/color/image_raw`: the connected device may not expose an
  RGB Camera profile. Use `rs-enumerate-devices` to confirm available streams.
- Low frame rate or dropped frames: disable unused streams and avoid running
  multiple viewers at the same time.
- Multiple cameras: pass `serial_no:=_<serial>` so the flight stack binds to the
  intended D435. Example: serial `831612073525` is launched as
  `serial_no:=_831612073525`.
