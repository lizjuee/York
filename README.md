# York
york小车的ros功能包，包括gmapping扫图和navigation导航

# 开发环境：
Ubuntu 18.04 + ros melodic(Jetson Nano) 

# 传感器：

深度摄像头

# 实验平台：

York 小车

# 这个demo一共有三个功能包。分别是：
1. hexros 小车的ros功能包
2. rplidar_ros 思岚A1雷达的启动功能包
3. mbot_navigation 扫图gmapping和导航navigation的功能包
4. darknet_ros YoloV3的识别功能包

# 指令合集
```
1.	roslaunch hexros node_vehicle.launch 启动小车
2.	roslaunch rplidar rplidar.launch 启动雷达
3.	roslaunch mbot_navigation gmapping_demo.launch 启动gmapping扫图
4.	roslaunch mbot_teleop mbot_teleop.launch 键盘控制小车
5.	rosrun map_server map_saver -f test 保存地图
6.	roslaunch mbot_navigation nav_cloister_demo.launch 启动导航
7.	roslaunch realsense2_camera rs_rgbd.launch 打开realsense
8.	roslaunch darknet_ros darknet_ros.launch 启动darknet_ros
```
