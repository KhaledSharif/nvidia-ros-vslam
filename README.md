# Visual Simultaneous Localization and Mapping (VSLAM)
Quickstart for GPU-Accelerated VSLAM in NVIDIA Isaac Sim (See [Prerequisites](https://nvidia-isaac-ros.github.io/getting_started/isaac_sim/index.html))

### Terminal A: Isaac Sim
```bash
# Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh
```

### Terminal B: Isaac ROS Visual SLAM Node
```bash
# Launch Isaac ROS Docker container and enter terminal
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && scripts/run_dev.sh

# Install Isaac ROS Visual SLAM package for ROS Humble
sudo apt-get install -y ros-humble-isaac-ros-visual-slam

# Launch Visual SLAM ROS node for Isaac Sim
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py
```
### Terminal C: RVIZ
```bash
# Run command from root of this repository
rviz2 -d ./rviz/isaac_sim.cfg.rviz
```
### Terminal D: Command Robot to Move
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```
### Debugging
```bash
ros2 topic list
```

```
/clock
/cmd_vel
/front_3d_lidar/point_cloud
/front_stereo_camera/imu/data
/front_stereo_camera/left_rgb/camerainfo
/front_stereo_camera/left_rgb/image_raw
/front_stereo_camera/right_rgb/camerainfo
/front_stereo_camera/right_rgb/image_raw
/left_stereo_camera/imu/data
/odom
/parameter_events
/rear_stereo_camera/imu/data
/right_stereo_camera/imu/data
/rosout
/tf
/tf_static
/visual_slam/imu
/visual_slam/status
/visual_slam/tracking/odometry
/visual_slam/tracking/slam_path
/visual_slam/tracking/vo_path
/visual_slam/tracking/vo_pose
/visual_slam/tracking/vo_pose_covariance
/visual_slam/vis/gravity
/visual_slam/vis/landmarks_cloud
/visual_slam/vis/localizer
/visual_slam/vis/localizer_loop_closure_cloud
/visual_slam/vis/localizer_map_cloud
/visual_slam/vis/localizer_observations_cloud
/visual_slam/vis/loop_closure_cloud
/visual_slam/vis/observations_cloud
/visual_slam/vis/pose_graph_edges
/visual_slam/vis/pose_graph_edges2
/visual_slam/vis/pose_graph_nodes
/visual_slam/vis/slam_odometry
/visual_slam/vis/velocity
```