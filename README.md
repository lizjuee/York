# York
the ros package of york agv，include gmapping and navigation

<img src="https://github.com/qq44642754a/mechanical-arm/blob/master/serial_test/media/run_service.png" width="400">
[![Watch the video](https://github.com/lizjuee/York/blob/master/media/video1.mp4)

# develop envirenment：
Ubuntu 18.04 + ros melodic(Jetson Nano) 

# sensor：

intel realsense d435

# platform：

York agv

# three package in this demo：
1. hexros (rospackage of york agv)
2. rplidar_ros (launch package of slam A1 lidar)
3. mbot_navigation ( gmapping and navigation package)
4. darknet_ros (identify package use YoloV3)

# Instruction set
```
1.	roslaunch hexros node_vehicle.launch (launch york agv)
2.	roslaunch rplidar rplidar.launch (launch lidar)
3.	roslaunch mbot_navigation gmapping_demo.launch (launch gmapping)
4.	roslaunch mbot_teleop mbot_teleop.launch (control york agv use keyboard)
5.	rosrun map_server map_saver -f test (save map)
6.	roslaunch mbot_navigation nav_cloister_demo.launch (launch navigation)
7.	roslaunch realsense2_camera rs_rgbd.launch (launch realsense)
8.	roslaunch darknet_ros darknet_ros.launch (launch darknet_ros)
```
