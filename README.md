A ROS 2 pipeline for autonomous semantic floor mapping using LiDAR-based SLAM, YOLO26 object detection, and spatial instance tracking. Motivated by commercial home robots such as the Roborock Saros Z70, the system detects and localises household objects on a 2D occupancy map while a reactive exploration node autonomously navigates the environment.

## Prerequisites
ROS 2 and System Dependencies

ROS 2 Humble (Ubuntu 22.04) is required.

```bash
sudo apt install ros-humble-rtabmap-ros
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-turtlebot3-simulations
sudo apt install ros-humble-realsense2-camera
```
## Python Dependencies

```bash
bashpip install ultralytics    
pip install "numpy<2"  
```

## Environment Variables
```bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=~/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
export ROS_DOMAIN_ID=0
source ~/turtlebot3_ws/install/setup.bash
```

## Building the Package

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select semantic_mapping
source install/setup.bash
```

## Running in Simulation

Launch the following in separate terminals in this order:

**Terminal 1 — Gazebo simulation:**
``` bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
```

**Terminal 2 — RTABMap SLAM:**
```bash
bashros2 launch rtabmap_launch rtabmap.launch.py \
    slam_mode:=true \
    use_sim_time:=true \
    subscribe_depth:=true \
    subscribe_rgb:=true \
    visual_odometry:=false \
    approx_sync:=true \
    approx_sync_max_interval:=0.05 \
    frame_id:=base_footprint \
    odom_topic:=/odom \
    rgb_topic:=/camera/image_raw \
    depth_topic:=/camera/depth/image_raw \
    camera_info_topic:=/camera/camera_info \
    qos:=1
```
**Terminal 3 — All semantic mapping nodes:**

```bash
ros2 launch semantic_mapping semantic_mapping.launch.py
```
This starts: depth_from_lidar, yolo_detector, sim_fusion_node, and random_explorer with staggered delays to allow RTABMap to initialise before exploration begins.

**Terminal 4 — RViz:**

```bash
ros2 run rviz2 rviz2
```
RViz Configuration
Add the following displays:
| Display | Topic | Notes |
|---------|-------|-------|
| Map | `/rtabmap/map` | Set fixed frame to `map` |
| MarkerArray | `/semantic_map/markers` | Orange spheres = detected objects |
| Image | `/detections/image` | YOLO annotated camera feed |
| LaserScan | `/scan` | LiDAR scan visualisation |

Set **View Type** to `TopDownOrtho` for the best occupancy map view.


## Running on Real Hardware (RealSense D435)

**Terminal 1 — RealSense camera:**
```bash
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    align_depth.enable:=true
```
 
**Terminal 2 — RTABMap SLAM:**
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    slam_mode:=true \
    subscribe_depth:=true \
    subscribe_rgb:=true \
    visual_odometry:=false \
    approx_sync:=true \
    approx_sync_max_interval:=0.05 \
    frame_id:=base_footprint \
    odom_topic:=/odom \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    qos:=1
```
 
**Terminal 3 — YOLO detector (real hardware topics):**
```bash
ros2 run semantic_mapping yolo_detector \
    --ros-args -p image_topic:=/camera/camera/color/image_raw
```
 
**Terminal 4 — Real fusion node:**
```bash
ros2 run semantic_mapping fusion_node
```
 
**Terminal 5 — Autonomous explorer (optional):**
```bash
ros2 run semantic_mapping random_explorer
```
