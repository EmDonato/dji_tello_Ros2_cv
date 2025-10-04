# DJI Tello ROS2 CV Integration

A ROS 2 workspace providing a **complete vision and control interface for the DJI Tello drone**, integrating:

* Real-time telemetry and odometry via `tello_driver_node`
* Gamepad teleoperation through `gamepad_tello_node`
* Object detection with **YOLOv8** (Python)
* **ArUco marker detection** with OpenCV (C++)
* TF broadcasting for front/down cameras (calibrated)

---

## 🚀 Overview

This project bridges the **DJI Tello** drone with **ROS 2** to enable full robotic perception and control capabilities.
It allows:

* Direct drone control via joystick/gamepad
* Real-time video streaming from front/down cameras
* Switching between cameras
* Person detection via YOLOv8
* Marker localization using OpenCV ArUco
* Continuous TF and odometry broadcasting for spatial awareness

The system is modular — each functionality (driver, vision, control) runs as a dedicated ROS 2 node.

---
## ⚙️ Project Structure

```
dji_tello_ROS2/
├── src/
|   ├── dji_tello_yolo/          # Yolo person identification node
│   ├── dji_tello_ros2/          # Driver node and gamepad node, there are also the interface lib for the minidrone and the launch file
│   ├── dji_tello_aruco/         # ArUco marker detection
│   └── dji_tello_interfaces/    # Custom service definitions
├── config/                      # Camera calibration files
├── install/
├── build/
└── log/
```

---
## 🧩 Dependencies

### ROS 2 Packages

* `rclcpp`
* `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`
* `cv_bridge`, `camera_info_manager`
* `tf2`, `tf2_ros`
* `image_transport` (optional, for camera optimization)

### Python

For the YOLOv8 node:

```bash
pip install ultralytics opencv-python cv-bridge
```

### System Dependencies

* OpenCV ≥ 4.5
* Python ≥ 3.8
* CMake ≥ 3.8
* Compatible with ROS 2 Humble

---

## 🛠️ Build Instructions

Clone the repository and build using **colcon**:

```bash
cd ~/ros2_ws/src
git clone git@github.com:EmDonato/dji_tello_Ros2_cv.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 🏃‍♀️ Running Example

Launch each node in separate terminals:

```bash

source install/setup.bash
ros2 launch dji_tello_ros2 launch_dji_tello_ros2.py

```

---

##  Node Summary

### `dji_tello_driver`

Handles communication with the drone:

* Publishes telemetry (IMU, odometry, TOF, battery)
* Streams video and camera info
* Provides flight commands through `/tello_command`

**Published Topics**

| Topic                | Type                     | Description                       |
| -------------------- | ------------------------ | --------------------------------- |
| `/battery`           | `std_msgs/Int32`         | Drone battery percentage          |
| `/odom`              | `nav_msgs/Odometry`      | Integrated position/velocity      |
| `/imu`               | `sensor_msgs/Imu`        | IMU orientation and angular rates |
| `/tello/image_raw`   | `sensor_msgs/Image`      | Camera stream                     |
| `/tello/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics                 |
| `/tf`                | `tf2_msgs/TFMessage`     | Drone → Camera transforms         |

**Service**

| Service          | Type                                | Command                                          | Description   |
| ---------------- | ----------------------------------- | ------------------------------------------------ | ------------- |
| `/tello_command` | `dji_tello_interfaces/srv/TelloCmd` | TAKEOFF, LAND, STREAMON, STREAMOFF, SWITCHCAMERA | Drone control |

---

### 🎮 `gamepad_tello_node`

Interfaces a joystick or Xbox-style controller for manual flight and vision toggling.

**Button Mapping**
This is mapped on a google stadia gamepad, be careful to adapt the configuration to your own gamepad

| Button  | Command                              | Description                      |
| :------ | :----------------------------------- | :------------------------------- |
| A       | `TAKEOFF`                            | Take off                         |
| B       | `LAND`                               | Land                             |
| X       | `STREAMON` / `STREAMOFF`             | Toggle video stream              |
| Y       | `SWITCHCAMERA`                       | Switch between front/down camera |
| D-Pad ↑ | `SHOWCAMERA`                         | Show camera stream               |
| D-Pad ↓ | `YOLO`                               | Toggle YOLOv8 detection          |
| D-Pad ← | `ARUCO`                              | Toggle ArUco detection           |
| L1 / R1 | Rotate left/right (yaw)              |                                  |
| L2 / R2 | down  /  up                          |                                  |
| L3 / R3 | pich  /  roll  (forward / lateral)   | Joysticks                        |

Publishes `cmd_vel_gamepad` messages to control the drone velocity.

---

### `yolo_person_detector` (Python)

Detects **persons** in real-time using **YOLOv8-nano**.

* Subscribes: `/tello/image_raw`
* Service: `/tello_yolo`
* Displays annotated frames in an OpenCV window.
* Toggles detection ON/OFF via service or D-Pad ↓.

---

### `tello_aruco_node`

Performs **ArUco marker detection** and broadcasts relative pose TFs.

* Supports camera switching (`front_camera` ↔ `down_camera`)
* Publishes `aruco → camera` transforms
* Service `/tello_aruco`:

  * `ARUCO` → toggle detection
  * `SWITCHCAMERA` → swap intrinsics dynamically

---

## 📷 Camera Calibration Files

Camera calibration YAMLs (OpenCV format) are located in:

```
src/dji_tello_ros2/config/
├── frontal_camera.yaml
├── down_camera.yaml
├── frontal_camera4ROS.yaml
└── down_camera4ROS.yaml
```

Both are loaded at runtime by the `tello_driver_node` and `tello_aruco_node`.
Switching camera updates the active calibration automatically.

---

## 🦯 TF Frames

| Parent       | Child                          | Description                      |
| :----------- | :----------------------------- | :------------------------------- |
| `odom_tello` | `base_link`                    | Drone odometry in world frame    |
| `base_link`  | `front_camera`                 | Front camera extrinsics          |
| `base_link`  | `down_camera`                  | Downward camera extrinsics       |
| `aruco`      | `front_camera` / `down_camera` | Marker-based camera localization |

---







## 👤 Author

Developed by **EmDonato**
Integration of computer vision and control for the DJI Tello using ROS 2 and OpenCV.
Licensed under the **MIT License**.
