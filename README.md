## Packages List
### Lucia
List of packages required to run Lucia

- **Motor & Encoder controller**
  - Repo:[lucia_controller](https://github.com/iHaruruki/lucia_controller.git)

- **Joy Stick Controller**
  - Repo:[joy_to_cmdvel](https://github.com/iHaruruki/joy_to_cmdvel.git)

- **Lucia URDF**
  - Reo:[lucia_description](https://github.com/iHaruruki/lucia_description.git)

- **LiDAR**
  - UTM-30LX-EW    
    - [UTM-30LX-EW](https://www.hokuyo-aut.co.jp/search/single.php?serial=146#program)        
    - Repo:[urg_node2](https://github.com/iHaruruki/urg_node2.git)
  - dual_laser_merger(ROS 2 Package to merge dual lidar scan data)
    - Rep:[dual_laser_merger](https://github.com/iHaruruki/dual_laser_merger.git)

- **Depth Camera**
  - Astra_pro(for Lucia)    
    - Repo:[ros2_astra_camera](https://github.com/iHaruruki/ros2_astra_camera.git)  
  - Astra_stereo_u3(for Mani)    
    - Repo:[OrbbecSDK_ROS2](https://github.com/iHaruruki/OrbbecSDK_ROS2.git)

- **SLAM**
  - slam_toolbox
    - Repo:[lucia_slam_toolbox](https://github.com/iHaruruki/lucia_slam_toolbox.git)
  - cartographer
    - Repo:[lucia_cartographer](https://github.com/iHaruruki/lucia_cartographer.git)

- **Navigation**
  - [lucia_navigation2](https://github.com/iHaruruki/lucia_navigation2.git)
- **MapStorage**
  - [MapStorage](https://github.com/iHaruruki/maps.git)

### Spina
List of packages required to run Spina
- **arm**
  - [spina_arm_controll](https://github.com/iHaruruki/spina_arm_controll.git)

- **kinematics**
- [spina_inverse_kinematics](https://github.com/iHaruruki/spina_inverse_kinematics.git)

### vital
- **vital measurement**
  - [lucia_vital](https://github.com/iHaruruki/lucia_vital.git)
- **vital calibration**
  - [lucia_vital_calibration](https://github.com/iHaruruki/lucia_vital_calibration.git)

- **vital display**
  - [lucia_vital_signs_display](https://github.com/iHaruruki/lucia_vital_signs_display.git)

## Command List
### Manual Control
#### Launch Lucia's motor and LiDAR
```shell
ros2 launch lucia_controller bringup.launch.py
```
#### Run Teleoperation Node
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### Joystick Control
Connect the DUALSHOCK4 to your computer
```shell
ros2 run joy joy_node
```
```shell
ros2 run joy_to_cmdvel joy_to_cmdvel_node
```
### SLAM
#### Launch Lucia's motor and LiDAR
```shell
ros2 launch lucia_controller bringup.launch.py
```
#### Run SLAM Nodes
```shell
ros2 launch lucia_slam_toolbox online_async_launch.py 
```
#### Run Teleoperation Node
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
*Start exploring and drawing the map.*
#### Save Map
The -f option specifies a folder location and a file name where files to be saved.
With the above command, map.pgm and map.yaml will be saved in the home folder ~/(/home/${username}).
```shell
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### Navigation
#### LaunchLucia's motor and LiDAR
```shell
ros2 launch lucia_controller bringup.launch.py
```
#### Run Navigation Nodes
```shell
ros2 launch lucia_navigation2 navigation2.launch.py map:=$HOME/ros2_ws/src/lucia_navigation2/map/map.yaml use_sim_time:=false
```
##### Estimate Initia Pose
1. Click the 2D Pose Estimate button in the RViz2 menu.
2. Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
3. Repeat step 1 and 2 until the LDS sensor data is overlayed on the saved map.
##### Set Navigation Goal
1. Click the Navigation2 Goal button in the RViz2 menu.
2. Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.

### spina_arm_controll
```shell
sudo chmod 777 /dev/ttyUSB0
```
```shell
ros2 run spina_arm_controll serial_controller_node
```
#### Set the overall angle to +90°
```shell
ros2 topic pub /angle_cmd std_msgs/msg/String "{ data: 'A0p-090' }" --once
```
### Lucia_vital_signs_display
**System that provides feedback on vital sign measurement results through voice guidance**    
YARP sound generator
```shell
yarpmanager --application /home/robot/repos/robot/script/ymanager/xml/applications/tutorial/tutorial_audio_3.xml
```
run arm controller node
```shell
ros2 run spina_arm_controll serial_controller_node
```
run vital controller node
```shell
ros2 run lucia_vital vital_controller_node
```
run vital audio guidance node
```shell
ros2 run lucia_vital_signs_display vital_audio_guidance_node 
```
#### goal announcement(Used when debugging Lucia_vital_signs_display)
```shell
ros2 topic pub \
  /navigate_to_pose/_action/status \
  action_msgs/msg/GoalStatusArray \
  "{status_list:
    [
      {
        goal_info:
          {
            stamp:    {sec: 0, nanosec: 0},
            goal_id:  {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]}
          },
        status: 4
      }
    ]
  }" \
  --once
```
