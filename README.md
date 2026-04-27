# RT2 Assignment 1 – Navigation with Action Server

## Overview
This project implements a simple robot navigation system in ROS 2 using **actions, components (plugins), and tf2** in C++.
The goal is to move a robot in a 2D environment to a target pose `(x, y, theta)` using an **action server**, without obstacle avoidance.
Both the **user interface (action client)** and the **navigation logic (action server)** are implemented as **components and executed in the same process** using **manual composition (no launch file)**.

---

## Features

- Send navigation goals `(x, y, theta)` from terminal
- Cancel active goal
- Continuous feedback from action server
- Robot motion using `/cmd_vel`
- Robot pose obtained using **tf2 transforms**
- Implemented entirely in **C++**

---

## System Architecture
```
User Input (terminal)
   ↓
UiClient (Action Client)
   ↓
NavServer (Action Server)
   ↓
/cmd_vel
   ↓
Robot (Gazebo)
   ↓
/odom → TF transform (odom → base_link)
   ↓
NavServer (feedback loop)
```
---

## Package Structure
```
rt2_nav_cpp/
├── action/
│   └── NavigateToPose.action
├── include/rt2_nav_cpp/
│   ├── nav_server_component.hpp
│   └── ui_client_component.hpp
├── src/
│   ├── nav_server_component.cpp
│   ├── ui_client_component.cpp
│   └── manual_container_main.cpp
├── CMakeLists.txt
└── package.xml
```
---

## Components

### NavServerComponent 

- Implements navigation logic
- Uses **tf2** to get robot pose (`odom → base_link`)
- Subscribes to `/odom` and broadcasts TF
- Publishes velocity commands to `/cmd_vel`
- Computes:
  - distance error
  - heading error
- Controls robot until:
  - goal is reached
  - or goal is canceled

---

### UiClient

- Reads input from terminal
- Sends goals to action server
- Supports:
  - goal input: `x y theta`
  - cancel command: `cancel`
- Displays:
  - feedback
  - result

---

## How to Run

### 1. Build workspace

```bash
cd ~/ros2_ws
colcon build
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2. Start robot (Gazebo)
```bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```

### 3. Run navigation system
```bash
ros2 run rt2_nav_cpp manual_container
```

### 4. Control robot (same terminal)
Send a goal:
```md
2 2 0
```
Cancel goal:
```md
cancel
```

## Topics Used
| Topic      | Type                | Description       |
| ---------- | ------------------- | ----------------- |
| `/cmd_vel` | geometry_msgs/Twist | velocity commands |
| `/odom`    | nav_msgs/Odometry   | robot odometry    |

## TF Frames

| Frame       | Description           |
| ----------- | --------------------- |
| `odom`      | fixed reference frame |
| `base_link` | robot frame           |

## Action Interface
### NavigateToPose.action
- Goal:
```md
float64 x
float64 y
float64 theta
```
- Feedback:
```md
float64 current_x
float64 current_y
float64 current_theta
float64 distance_remaining
float64 heading_error
```
- Result:
```md
bool success
string message
```

## Debug / Verification

- Check velocity commands:
```bash
ros2 topic echo /cmd_vel
```
- Check robot pose:
```bash
ros2 topic echo /odom
```
- Check TF transform:
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
## Notes
- This implementation does NOT include obstacle avoidance, as required by the assignment.
- The robot may collide if obstacles are present.
- Control is based on geometric error (no path planning).
---
### External Dependency

Clone the robot package:

```bash
cd ~/ros2_ws/src
git clone -b rt2 https://github.com/CarmineD8/bme_gazebo_sensors.git
```
---
## Requirements
- Ubuntu 24.04 (WSL or native)
- ROS2 Jazzy
- bme_gazebo_sensors package
---

## Author: Endri Gjinaj
