# 🧠 SCMS_TurtleBot — Warehouse Caddy Bot (ROS 2)

The **Warehouse Caddy Bot** is a human-following TurtleBot system designed to autonomously assist workers in warehouse and logistics environments.  
Using **RGB-D sensing** and **ROS 2 Humble**, the robot detects and tracks a person wearing high-visibility clothing, maintaining a safe following distance of ~1 m.  
The system integrates perception, distance estimation, and control through a modular multi-node ROS 2 architecture, capable of running in both **Gazebo simulation** and on the **real TurtleBot3 platform**.

---

## 📘 Project Overview

| Component | Description |
|------------|-------------|
| **Project ID** | 01 — TurtleBot Following a Person |
| **Team Name** | Group 13 |
| **Project Title** | Warehouse Caddy Bot |
| **Platform** | TurtleBot3 (Burger / Waffle Pi) |
| **Framework** | ROS 2 Humble Hawksbill + Python 3.10 (virtual environment) |
| **Sensor** | RGB-D Depth Camera (e.g. Intel RealSense, Orbbec Astra, Kinect v2) |
| **Environment** | Warehouse / Logistics |
| **Input Topics** | `/camera/color/image_raw`, `/camera/depth/image_raw` |
| **Output Topics** | `/cmd_vel`, `/human_position` |
| **Goal** | Detect and follow a human wearing high-visibility clothing at ~1 m distance |

---

## 🏭 Application Context

The system functions as an **autonomous mobile assistant** in warehouse or factory settings.  
It follows workers wearing high-visibility vests, carrying tools or parts, and reducing the need for manual carts or trolleys.  
This improves workflow efficiency, safety, and ergonomics — particularly in **GPS-denied indoor environments** where hands-free operation is vital.  
The combined RGB and depth sensing allows reliable detection even under variable lighting or cluttered backgrounds.

---

## 🎯 Project Scope

**In Scope**
- Detect human wearing a Hi-Vis vest using RGB-D data  
- Estimate distance between robot and human  
- Implement a control system to follow at a safe distance  

**Out of Scope**
- Feature detection / semantic segmentation  
- Uneven terrain traversal  
- Multi-person tracking  
- Collision or obstacle avoidance  

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
- Implements a **PID-based** control loop to maintain 1.0 m distance  
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
cd ~/mnt/c/Users/micah/Documents/SCMS_turtlebot
python3 -m venv .venv
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select depth_follower
source install/setup.bash
```

## 🚀 Launch Instructions

### Terminal 1 — Bring Up TurtleBot3
**Simulation**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
**Physical Robot**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### Terminal 2 — Launch the Depth Follower
```bash
cd ~/mnt/c/Users/micah/Documents/SCMS_turtlebot (for me)
source .venv/bin/activate
source install/setup.bash
ros2 launch depth_follower follower.launch.py
```

## 🔍 Verification
```bash
ros2 topic list
ros2 node list
ros2 topic echo /cmd_vel
```
### Expected topics:
``` bash
/camera/depth/image_raw
/cmd_vel
/depth_follower
```

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

