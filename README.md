<div align="center">

# 🤖 Lucia & Spina System  
*A Modular Mobile Base + Arm + Vital Sensing Integration*  

[![ROS 2 Distro - Humble](https://img.shields.io/badge/ROS2-Humble-1f425f.svg)](https://docs.ros.org/)
[![Stars](https://img.shields.io/github/stars/iHaruruki?style=social)](https://github.com/iHaruruki)
</div>
---

## 🧩 Component Overview (構成概要)

| Device | Description |
|----------|----------------|
| Lucia (Base) | Mobile base: motors, LiDAR, depth, SLAM, navigation |
| Spina (Arm) | Arm: serial control & inverse kinematics |
| Vital | Vital measurement |

### System Flow Diagram

```mermaid
flowchart LR
  subgraph Base[Lucia Base]
    MC[Motor Controller]
    LD[LiDAR]
    DC[Depth Camera]
  end
  subgraph SLAM_NAV[SLAM & Nav]
    SLAM[slam_toolbox]
    NAV[Navigation2]
  end
  subgraph ARM[Spina Arm]
    ARM_CTRL[Arm Serial Node]
    IK[IK Solver]
  end
  subgraph VITAL[Vital System]
    VM[Vital Controller]
    VC[Calibration]
    VA[Audio Guidance]
  end
  LD --> SLAM --> NAV
  DC --> SLAM
  MC --> NAV
  NAV --> MC
  VM --> VA
  NAV -.goal status.-> VA
  ARM_CTRL --> VM
```

---

## 📦 Package Matrix

| Domain | Function (EN) | Repository |
|--------|---------------|------------|
| Base | Motor & Encoder Controller | [lucia_controller](https://github.com/iHaruruki/lucia_controller) |
| Base | Joystick to cmd_vel | [joy_to_cmdvel](https://github.com/iHaruruki/joy_to_cmdvel) |
| Base | URDF / Description | [lucia_description](https://github.com/iHaruruki/lucia_description) |
| Sensing | LiDAR Driver | [urg_node2](https://github.com/iHaruruki/urg_node2) |
| Sensing | Dual LiDAR Merger | [dual_laser_merger](https://github.com/iHaruruki/dual_laser_merger) |
| Sensing | Depth Camera | [ros2_astra_camera](https://github.com/iHaruruki/ros2_astra_camera) |
| Sensing | Depth Camera | [OrbbecSDK_ROS2](https://github.com/iHaruruki/OrbbecSDK_ROS2) |
| Mapping | slam_toolbox | [lucia_slam_toolbox](https://github.com/iHaruruki/lucia_slam_toolbox) |
| Mapping | Cartographer | [lucia_cartographer](https://github.com/iHaruruki/lucia_cartographer) |
| Navigation | Navigation2 | [lucia_navigation2](https://github.com/iHaruruki/lucia_navigation2) |
| Maps | Map Storage | [maps](https://github.com/iHaruruki/maps) |
| Arm | Arm Serial Control | [spina_arm_controll](https://github.com/iHaruruki/spina_arm_controll) |
| Arm | Inverse Kinematics | [spina_inverse_kinematics](https://github.com/iHaruruki/spina_inverse_kinematics) |
| Vital | Vital Measurement | [lucia_vital](https://github.com/iHaruruki/lucia_vital) |
| Vital | Calibration | [lucia_vital_calibration](https://github.com/iHaruruki/lucia_vital_calibration) |
| Vital | Display | [lucia_vital_signs_display](https://github.com/iHaruruki/lucia_vital_signs_display) |
---

## 🛠 Environment
:watch: Coming soon
---

## :video_game: A. Manual Control mode

Bring up base + control via keyboard or joystick.
```bash
ros2 launch lucia_controller bringup.launch.py
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
```bash
ros2 run joy joy_node
```
```bash
ros2 run joy_to_cmdvel joy_to_cmdvel_node
```
---

## 🗺️ B. SLAM (Map Building)
  
Explore the environment to build a map.
```bash
ros2 launch lucia_controller bringup.launch.py
ros2 launch lucia_slam_toolbox online_async_launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*Start exploring and drawing the map.*
The -f option specifies a folder location and a file name where files to be saved.
With the above command, map.pgm and map.yaml will be saved in the home folder ~/(/home/${username}).
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
Outputs: `~/map.yaml`, `~/map.pgm`
---

## 🧭 C. Navigation (Using Saved Map)

1. Launch:
   ```bash
   ros2 launch lucia_controller bringup.launch.py
   ros2 launch lucia_navigation2 navigation2.launch.py \
     map:=$HOME/ros2_ws/src/lucia_navigation2/map/map.yaml \
     use_sim_time:=false
   ```
2. 初期姿勢推定 / Initial Pose (RViz2 で 2D Pose Estimate)  
3. 目標設定 / Send Navigation Goal (Navigation2 Goal ボタン)

---

## 🦾 D. Spina Arm Control

```bash
sudo chmod 777 /dev/ttyUSB0  # or add to dialout group
ros2 run spina_arm_controll serial_controller_node
# 例: 全体角度 +90° / Example command
ros2 topic pub /angle_cmd std_msgs/msg/String "{ data: 'A0p-090' }" --once
```
---

## 💓🔊 E. Vital Signs Display System

EN: Run vital sensing and audio feedback pipeline.
```bash
# (Optional) YARP audio
yarpmanager --application /home/robot/repos/robot/script/ymanager/xml/applications/tutorial/tutorial_audio_3.xml
ros2 run spina_arm_controll serial_controller_node
ros2 run lucia_vital vital_controller_node
ros2 run lucia_vital_signs_display vital_audio_guidance_node
```

---

## 🧪 Debug
### Navigation / Simulate Nav Success
```bash
ros2 topic pub \
  /navigate_to_pose/_action/status \
  action_msgs/msg/GoalStatusArray \
  "{status_list:
    [
      {
        goal_info:
          { stamp: {sec: 0, nanosec: 0},
            goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]} },
        status: 4
      }
    ]
  }" --once
```
`status: 4` = SUCCEEDED

---

## 🛠 Troubleshooting
:clock: Coming soon