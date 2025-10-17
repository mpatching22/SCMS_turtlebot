# 🏭 warehouse_env — Gazebo Warehouse Simulation Environment
This package builds and launches the warehouse world for our TurtleBot project.
It includes:
- 📦 A warehouse scene (floor, shelves, boxes)
- 👷 Hi-vis person / moving object placeholder 
- 🌍 Gazebo (Ignition) world for testing perception nodes 

---

## 🚀 Quick Launch Instructions

### 1. Clone & Build

```bash
cd ~/Documents/SCMS_turtlebot/src
git clone <repo_or_copy_folder_here>

cd ~/Documents/SCMS_turtlebot
colcon build --symlink-install
source install/setup.bash
```

### 2. Set Model Path (do this each new terminal)

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/src/warehouse_env/models
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(pwd)/src/warehouse_env/models
```
(Add both lines to your ~/.bashrc for convenience.)

### 3. Launch in Gazebo
```bash
ign gazebo src/warehouse_env/worlds/warehouse.world.sdf
```
You should see:
- A concrete floor slab
- Four shelves and two boxes
- Ambient lighting (bright scene)
If everything looks correct, the environment is ready for the TurtleBot to spawn.


# 🧰 Common Issues & Fixes

| Problem | Cause | Fix |
|------------|-------------| -------------|
| **Models missing** | Resource path not exported | Ensure both $GZ_SIM_RESOURCE_PATH and $IGN_GAZEBO_RESOURCE_PATH include warehouse_env/models |
| **Gazebo command not found** | ROS Gazebo not installed | sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge |
| **Permission error: /run/user/1000/** | WSL temp directory permission mismatch | Run: sudo chmod 700 /run/user/1000/ |

---

# 💡 Developer Notes

- World file: warehouse_env/worlds/warehouse.world.sdf
- Models: warehouse_env/models/ (shelf, pallet_box, floor, hi_vis_person)
- Launch file: warehouse_env/launch/warehouse_world.launch.py
- Renderer fallback (if dark):


```bash
IGN_RENDER_ENGINE=ogre ign gazebo ...
```
