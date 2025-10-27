# 🧠 SCMS_TurtleBot — Warehouse Caddy Bot (ROS 2)

The Warehouse Caddy Bot is an autonomous human-following mobile robot developed using ROS 2 Humble on a TurtleBot3 Waffle Pi platform.
It leverages RGB-D sensing, LiDAR safety, and a modular multi-node control system to follow a worker wearing high-visibility clothing at approximately 1 m distance — both in Gazebo simulation and on real hardware.

---

## 📘 Project Overview

| Component | Description |
|------------|-------------|
| **Project ID** | 01 — TurtleBot Following a Person |
| **Team Name** | Group 13 |
| **Project Title** | Warehouse Caddy Bot |
| **Platform** | TurtleBot3 (Burger / Waffle Pi) |
| **Framework** | ROS 2 Humble Hawksbill + Python 3.10 (virtual environment) |
| **Sensor** | RGB-D Depth Camera, 2D LiDAR |
| **Environment** | Warehouse / Logistics |
| **Input Topics** | `/camera/color/image_raw`, `/camera/depth/image_raw`, `/scan` |
| **Output Topics** | `/cmd_vel`, `/human_position` |
| **Goal** | Detect and follow a human wearing high-visibility clothing at ~1 m distance |

---

## 🏭 Application Context

The system acts as a **mobile warehouse assistant**, autonomously transporting goods or tools by following a worker.
Operating in **GPS-denied indoor environments**, it supports ergonomic, hands-free workflows and enhances occupational safety.
By combining **RGB-D perception** with **LiDAR-based safety checks**, the robot achieves reliable human tracking under variable lighting, clutter, and partial occlusions.

---

## 🎯 Project Scope

**In Scope**
- Detect human wearing a Hi-Vis vest using RGB-D data  
- Estimate distance between robot and human  
- Implement a control system to follow at a safe distance  

**Out of Scope**
- Full semantic segmentation or object classification
- Multi-person tracking / dynamic obstacle avoidance
- Outdoor / uneven terrain navigation

Focus remains on **core perception–control integration** rather than full autonomous navigation.

---

## 🧩 Deliverables

### 1. RGB-D Human Detection Node
- Fuses RGB and depth data to identify a person wearing high-visibility clothing  
- Uses HSV colour segmentation + depth extraction to determine 3-D position  
- Publishes to `/human_position` for downstream modules  

### 2. Distance Estimation Module
- Computes Euclidean distance between TurtleBot and detected human  
- Publishes continuous distance and lateral offset readings  
- Logs distance error and latency for evaluation  

### 3. Control System Node
- Implements a **P-based** control loop to maintain 1.0 m distance  
- Publishes velocity commands to `/cmd_vel` with smoothing + safety limits  
- Visualises control behaviour in RViz2  

### 4. Environment Differentiation
- Differentiates humans from static obstacles using combined RGB-D profiles  
- Ignores shelves and background planes  

### 5. System Integration and Launch Framework
- Unified `launch.py` connecting all nodes for easy execution  
- Supports both **Gazebo simulation** and **real robot deployment**  

### 6. Data Logging and Evaluation
- Records metrics: following-distance accuracy, response time, reliability  
- Enables quantitative analysis in the final report  

---

## ⚙️ Environment Setup

**Requirements**
- Ubuntu 22.04  
- ROS 2 Humble Hawksbill  
- Gazebo Fortress  
- Python 3.10  
- `rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `numpy`, `opencv-python`

**Setup Commands**
```bash
cd ~/turtlebot_ws/src
git clone https://github.com/<your-repo>/SCMS_turtlebot.git
cd ~/turtlebot_ws
python3 -m venv .venv
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select depth_follower
source install/setup.bash
```

## 🚀 Launch Instructions

### 🧪 Simulation in Gazebo

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot_ws/src/SCMS_turtlebot/src/depth_follower/models
ros2 launch depth_follower depth_follower.launch.py
```

### 🤖 Real Robot (Waffle Pi — ros2 turtlebot14)

**SSH Connection**
```bash
ssh ubuntu@192.168.0.214
```

**Launch Robot Bringup**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
```


**Run Follower Node**
```bash
ros2 launch depth_follower follower.launch.py
```

**Manual Movement Test**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}"
```

**Reboot Robot**
```bash
sudo reboot
```

**Start Camera Node**
```bash
ros2 run v4l2_camera v4l2_camera_node
```

---

## 🔍 Verification Checklist

```bash
ros2 topic list
ros2 node list
ros2 topic echo /cmd_vel
```

**Expected Topics:**
```bash
/camera/color/image_raw
/camera/depth/image_raw
/scan
/cmd_vel
/human_position
```
✅ Vest detected logs appear in console
✅ Robot maintains ~1 m distance and centres target
✅ LiDAR safety stops trigger correctly

---

## 🧠 Learning Objectives

- Apply ROS 2 frameworks for perception + control integration  
- Use RGB-D sensors to estimate human position and movement  
- Transition from simulation to real-world hardware  
- Implement and evaluate a complete robotic control pipeline  

---

## 🧱 Resources & Strategies

- UTS Mechatronics Lab for hardware testing  
- Weekly team coordination via GitHub & task planning  
- Literature: Soares et al. (2021); Liu et al. (2022)  
- Regular consultation with supervisor (Dominik Slomma)  

---

## 🧾 Assessment Criteria

| Grade | Description |
|--------|-------------|
| **F (Fail)** | No ROS nodes implemented / simulation incomplete |
| **P (Pass)** | One node (e.g. RGB-D detection) working in Gazebo |
| **C (Credit)** | Detection + distance modules functional in simulation |
| **D (Distinction)** | Full perception-control pipeline in Gazebo; smooth following |
| **HD (High Distinction)** | Full hardware integration with real TurtleBot3; reliable real-time tracking with RViz2 visualisation + logged metrics |

---

## 👥 Team Roles

| Member | Role |
|---------|------|
| **Micah Patching** | System architecture, ROS 2 integration, GitHub repo setup |
| **Joshua Chin** | Human-following algorithm (RGB-D detection + tracking) |
| **Wil Coxon** | Depth camera integration & point cloud processing |
| **Liam Davis** | TurtleBot control node & motion tuning |

_All roles are flexible to ensure balanced collaboration and consistent progress._

---

## 💻 Required Resources

### Software
- Ubuntu 22.04  
- ROS 2 Humble  
- Gazebo Fortress  
- Python 3.10  
- Standard ROS 2 libraries  

### Hardware
- TurtleBot3 with Intel RealSense camera  
- Ubuntu-compatible PC (for ROS 2 and Gazebo)  

### Other
- GitHub for version control  
- UTS Mechatronics Lab for testing and demo  



