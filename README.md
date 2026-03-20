# 🤖 Line Following Robot — ROS 2 + Gazebo

A ROS 2 differential-drive robot that autonomously follows a black line on a white surface using a downward-facing camera, PID control, and real-time image processing. Simulated entirely in Gazebo Classic.

---

## 📽️ Demo Video

ADDED ABOVE 
---

## 📁 Project Structure

```
line_following_robot/
├── launch/
│   └── line_follower.launch.py      # Launches Gazebo + robot + line follower node
├── line_following_robot/
│   ├── __init__.py
│   └── line_follower.py             # Main line-following logic (PID controller)
├── resource/
│   └── line_following_robot
├── test/
├── urdf/
│   ├── one_robot.xacro              # Robot description (links, joints, inertia)
│   └── one_robot.gazebo             # Gazebo plugins (diff drive + camera)
└── worlds/
    └── line_track_world.sdf         # Rectangular track world
```

---

## ⚙️ How It Works

### 1. Sensor Input
The robot has a forward-facing camera (`/camera/image_raw`) mounted at the front of the chassis, angled slightly downward (`pitch = 0.32 rad`) to view the track ahead.

### 2. Image Processing (`process_image_simple`)
Each frame is processed as follows:

```
Raw Frame (640×480)
      ↓
Crop bottom ROI (last 100 px — where the line is closest)
      ↓
Convert BGR → Grayscale
      ↓
Threshold (THRESH_BINARY_INV, threshold=100)  ← dark line on white floor
      ↓
Find contours → pick largest contour (area > 150 px²)
      ↓
Compute centroid X using image moments
```

The centroid X coordinate represents where the line is, relative to the camera frame.

### 3. PID Control (`simple_line_following`)
The error is computed as the difference between the detected line centroid and the image centre:

```
error = line_center_x − (image_width / 2)
```

A PID controller computes the angular correction:

```
angular_z = −(Kp·e + Ki·∫e·dt + Kd·Δe)
```

| Parameter | Value |
|-----------|-------|
| `Kp`      | 0.05  |
| `Ki`      | 0.0001|
| `Kd`      | 0.5   |
| Linear speed | 0.2 m/s |
| Error threshold (dead zone) | ±50 px |
| Integral clamp | ±50 |

If no line is detected, the robot spins in the direction of the last known error to search for the line.

### 4. Robot Platform

| Property | Value |
|----------|-------|
| Base dimensions | 0.6 × 0.4 × 0.2 m |
| Wheel radius | 0.1 m |
| Wheel separation | 0.45 m |
| Drive type | Differential drive (front wheels active) |
| Camera FOV | 60° horizontal |
| Camera resolution | 640 × 480 @ 30 Hz |

---

## 🗺️ World

The simulation world (`line_track_world.sdf`) is a **white ground plane (20 × 15 m)** with a closed rectangular black-line track:

| Segment      | Pose (x, y) | Length |
|--------------|-------------|--------|
| Bottom edge  | (0, −2)     | 6 m    |
| Top edge     | (0, +2)     | 6 m    |
| Left edge    | (−3, 0)     | 4 m    |
| Right edge   | (+3, 0)     | 4 m    |

The robot spawns at `(0, 0, 0)` facing the bottom edge (yaw = −90°).

---

## 🚀 Getting Started

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (gazebo_ros)
- Python packages: `cv2`, `cv_bridge`, `rclpy`

### Build

```bash
cd ~/ros2_ws/src
# (clone this repo here)
cd ~/ros2_ws
colcon build --packages-select line_following_robot
source install/setup.bash
```

### Run

```bash
ros2 launch line_following_robot line_follower.launch.py
```

This will:
1. Start Gazebo with the rectangular track world
2. Spawn the robot at the origin
3. Start the `robot_state_publisher`
4. Launch the `line_follower` node

---

## 🔧 Tuning Parameters

The PID gains and speed can be adjusted at runtime:

```bash
ros2 param set /Line_Follower linear_speed 0.15
ros2 param set /Line_Follower angular_speed 0.5
```

Or change defaults in `line_follower.py`:

```python
self.declare_parameter('linear_speed', 0.2)
self.declare_parameter('angular_speed', 0.5)
self.kp = 0.05
self.ki = 0.0001
self.kd = 0.5
```

---

## 🐛 Debugging

The node logs status every 2 seconds:

```
[Line_Follower]: === STATUS ===
[Line_Follower]: Images received: 120
[Line_Follower]: Camera working: True
[Line_Follower]: Line detected: True
[Line_Follower]: Last line center: 318
```

Detailed per-frame logs appear every 30 frames. To monitor topics:

```bash
# Check camera feed
ros2 topic echo /camera/image_raw --no-arr

# Check velocity commands
ros2 topic echo /cmd_vel

# Visualise camera in RViz2
rviz2
```

---

## 📦 Dependencies

| Package | Purpose |
|---------|---------|
| `rclpy` | ROS 2 Python client |
| `sensor_msgs` | Image message type |
| `geometry_msgs` | Twist (velocity) message type |
| `cv_bridge` | ROS ↔ OpenCV image conversion |
| `opencv-python` | Image processing |
| `gazebo_ros` | Simulation plugins |
| `robot_state_publisher` | TF tree publishing |
| `xacro` | URDF macro processing |

---

## 📜 License

MIT License — free to use, modify, and distribute.

---

## 🙌 Acknowledgements

Built with ROS 2 Humble, Gazebo Classic, and OpenCV. Inspired by classical line-following robot competitions.
