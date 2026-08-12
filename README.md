# Interplanetar 2026 Recruitment Submission

Two tasks completed: **Task 1** (Voice-Controlled Quadrotor & Telemetry) and
**Task 2** (Autonomous Navigation & Vision, TurtleBot4).

Tested on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.

## Prerequisites

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-turtlebot4-simulator \
    ros-jazzy-turtlebot4-desktop ros-jazzy-ros-gz ros-jazzy-cv-bridge \
    python3-colcon-common-extensions portaudio19-dev python3-tk

pip3 install --break-system-packages websockets opencv-python numpy \
    "numpy<2" SpeechRecognition pyaudio faster-whisper
```

> **NVIDIA hybrid-graphics laptops:** if Gazebo renders incorrectly or sensor
> data looks wrong (e.g. LiDAR returning near-minimum range everywhere), force
> GPU render offload before launching Gazebo:
> ```bash
> export __NV_PRIME_RENDER_OFFLOAD=1
> export __GLX_VENDOR_LIBRARY_NAME=nvidia
> export __VK_LAYER_NV_optimus=NVIDIA_only
> ```

## Build

```bash
mkdir -p ~/ros2_ws/src
cp -r src/questions src/quadrotor_control ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Task 2: TurtleBot4 Navigation & Vision

**Terminal 1** — launch the simulation (spawns TurtleBot4 in the `level3`
world and starts the WebSocket waypoint broadcaster):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch questions main_assignment.launch.py
```

**Terminal 2** — run the navigation node (connects to the WebSocket, drives
through all waypoints, reactively backs away/turns/clears any obstacle in
its path):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run questions waypoint_navigator
```

**Terminal 3** — run the vision node (waits for navigation to finish, then
identifies the largest of the three colored spheres using depth-based real
size estimation, not just pixel area):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run questions vision_node
```

### Bonus: SLAM

With Terminals 1 and 2 already running, add a fourth terminal to build a
live map while the robot navigates:
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot4_navigation slam.launch.py
```

To visualize the map being built in real time:
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

To save the finished map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/interplanetar_map
```

No custom code is needed — `slam_toolbox` (via TurtleBot4's official launch
files) builds the map from the same `/scan` and odometry data the navigator
is already using while driving the waypoint route.

### Implementation notes
- **Navigation**: go-to-goal controller (proportional heading + distance
  control) driven by ground-truth pose (`/sim_ground_truth_pose`).
- **Obstacle handling**: reactive, not path-planned. On detecting something
  within 0.2m directly ahead (LiDAR `/scan`, corrected for a measured 90°
  sensor mounting offset), the robot backs up 0.1m, turns 90° right, drives
  forward 0.5m, then resumes toward the current waypoint. Maneuver
  completion is driven by measured odometry (position/yaw change), not
  elapsed time, so it works correctly regardless of simulation speed.
- **Vision**: HSV color segmentation + contour detection to find each
  sphere, then uses the depth camera and camera intrinsics (pinhole model)
  to convert pixel radius to true physical radius, so the reported "largest"
  sphere is correct regardless of the robot's exact final distance from each
  one.
- **Known limitation**: obstacle avoidance is reactive (fixed maneuver), not
  full path planning — it handles the known obstacle in this world but isn't
  guaranteed to navigate arbitrary obstacle layouts.

---

## Task 1: Voice-Controlled Quadrotor & Telemetry

**Terminal 1** — launch Gazebo with the X3 quadrotor and the ROS2↔Gazebo
bridge:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export __NV_PRIME_RENDER_OFFLOAD=1   # if needed, see above
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
ros2 launch quadrotor_control quadrotor_sim.launch.py
```

**Terminal 2** — voice control node (listens on the system microphone,
transcribes locally with Whisper, maps recognized words to `Twist`
commands):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run quadrotor_control voice_control_node
```
Voice commands: `forward`, `backward`/`back`, `left`, `right`, `up`, `down`,
`stop`.

**Terminal 3** — telemetry dashboard + manual control GUI:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run quadrotor_control telemetry_gui_node
```

### Implementation notes
- **Simulated drone**: Gazebo Harmonic's official X3 multicopter example
  world, controlled via the native `MulticopterVelocityControl` plugin,
  bridged to ROS2 (`geometry_msgs/Twist` in, `nav_msgs/Odometry` out) via
  `ros_gz_bridge` — avoids Gazebo Classic entirely, which `sjtu_drone`
  (the common alternative) depends on and which Jazzy no longer supports.
- **Voice recognition**: local Whisper (`faster-whisper`, `small` model,
  CPU/int8) instead of a cloud API — more robust to accent variation and
  works fully offline.
- **Dual-mode switching**: the GUI publishes the active mode
  (`voice`/`manual`) on `/control_mode`. Both the voice node and the GUI's
  manual buttons publish to the same `Twist` topic, but each only acts when
  it's actually their turn, gated by the shared mode.
- **GUI/ROS2 integration**: Tkinter owns the main thread (required by the
  GUI toolkit); `rclpy.spin_once()` is called periodically via Tkinter's own
  `.after()` scheduler, interleaving ROS2 message processing into the
  existing event loop rather than using a separate thread.
- **Not implemented**: swarm/leader-follower bonus (out of scope for this
  submission).

---

## Docker

```bash
xhost +local:docker
docker compose build
docker compose run --rm interplanetar
```

Inside the container, follow the same `ros2 launch` / `ros2 run` commands
from the sections above. Open extra shells into the running container with:
```bash
docker compose exec interplanetar bash
```

**Caveat:** GUI (Gazebo, the Tkinter dashboard) and microphone access require
host passthrough (X11 + ALSA device mounts), configured for Linux hosts in
`docker-compose.yml`. This is inherently less portable than running natively
— if GUI windows don't appear, confirm `xhost +local:docker` was run on the
host first, and that `$DISPLAY` is set.

## Repository structure

```
src/
├── questions/              # Task 2 - TurtleBot4 navigation & vision
│   ├── questions/
│   │   ├── waypoint_navigator.py
│   │   ├── vision_node.py
│   │   └── websocket_broadcaster.py
│   ├── launch/main_assignment.launch.py
│   └── level3.sdf
└── quadrotor_control/       # Task 1 - Voice-controlled quadrotor
    ├── quadrotor_control/
    │   ├── voice_control_node.py
    │   └── telemetry_gui_node.py
    ├── launch/quadrotor_sim.launch.py
    └── worlds/quadrotor_world.sdf
```
