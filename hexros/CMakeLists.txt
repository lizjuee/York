cmake_minimum_required(VERSION 3.0.2)
project(hexros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_GLIBCXX_USE_C99=1)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(

)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_vehicle 
src/node_vehicle.cpp 
src/comm_serial_linux.cpp 
src/comm_transform.cpp 
src/vehicle_transform.cpp
)


target_link_libraries(node_vehicle
   ${catkin_LIBRARIES}
 )

