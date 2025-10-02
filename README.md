# SCMS_turtlebot — Depth Camera Person Follower (ROS 2)

This repository hosts the **SCMS TurtleBot project**, where our goal is to enable a TurtleBot to follow a human **using the depth camera** (RGB-D sensor).  
The system runs on ROS 2 (Humble) and uses simple image processing on depth frames to detect and track the closest person, then generates velocity commands to follow at a safe distance.

---

## 📌 Project Overview
- **Platform:** TurtleBot (Burger/Waffle Pi) running ROS 2 Humble  
- **Sensor:** Depth camera (e.g. RealSense, Orbbec Astra, Kinect v2)  
- **Input:** Depth image topic `/camera/depth/image_raw`  
- **Output:** Velocity commands on `/cmd_vel`  
- **Goal:** Detect the closest person in front of the robot and follow at ~1 m distance.  

---

## 🚀 Quickstart

# Build with colcon
colcon build --packages-select depth_follower depth_follower_bringup
. install/setup.bash
