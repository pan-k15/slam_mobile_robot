# SLAM Mobile Robot
2WD mobile robot with slam toolbox

## Features

- Differential drive with two powered wheels  
- Odometry publishing (`/odom`) and velocity command subscription (`/cmd_vel`)  
- 2D LiDAR sensor input (`/scan`) for mapping  
- Online SLAM with slam_toolbox  
- Map visualization with RViz2

## How to run
**Run simulation**
``` 
ros2 launch slambot_pkg sim.launch.py 
```
### Mapping
**Run SLAM toolbox**
```
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/pan/Documents/portfolio/slam_toolbox_robot/ros2_ws/src/slam_mobile_robot/slambot_pkg/config/mapper_params_online_async.yaml use_sim_time:=true
```
**Run Teleop**
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```
## Navigation
**Run**
```
ros2 run slambot_pkg cmd_vel_relay
```
***RVIZ2***
```
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```
**Run Localization**
```
ros2 launch slambot_pkg localization_launch.py
```
**Run Navigation**
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
---
## To do
- [x] Simulation
- [x] Mapping
- [x] Localization
- [x] Navigation
- [ ] Computer vision

---

#### Support me

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/P5P71JL5X3)

<a href="https://www.buymeacoffee.com/pan.tech" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" style="height: 60px !important;width: 217px !important;" ></a>
