//////////////////////////////////////////////////
//ros package for vehicle V1.0
//
//////////////////////////////////////////////////
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include <comm_serial_linux.h>
#include <vehicle_transform.h>
#include <comm_transform.h>

static CommSerialLinux m_comm_serial;
static CommTransform m_comm_transform;
static VehicleTransform m_vehcile_transform;
/*------------------------------------------------------------------------------------------------------------------
 * name: CallbackcVel
 * detail: Callback for cmd_vel
 -----------------------------------------------------------------------------------------------------------------*/
void CallbackcVel(const geometry_msgs::Twist& cmd_vel)
{
  m_comm_serial.SerialDataSend(m_comm_transform.StdVehiclemove(cmd_vel.linear.x*1000,cmd_vel.linear.y*1000,cmd_vel.angular.z*1000));
}

/*------------------------------------------------------------------------------------------------------------------
 * name: main
 * detail:
 -----------------------------------------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    system("gnome-terminal -x bash -c 'source /opt/ros/melodic/setup.bash;roscore'&");

    ros::init(argc, argv, "node_vehicle");
    ros::NodeHandle ros_node;
    ros::Rate rate_ms(50);//20ms
    ros::Rate rate_s(1);//1s
    ros::Subscriber sub_turtle_vel = ros_node.subscribe("turtle1/cmd_vel", 1000, CallbackcVel);
    ros::Subscriber sub_vel = ros_node.subscribe("cmd_vel", 1000, CallbackcVel);
    ros::Publisher pub_odom = ros_node.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Time current_time, last_time;
    tf::TransformBroadcaster odom_broadcaster;
    vector<unsigned char>recv_buff;
    //wait serial open/////////////////////////////////////////////////////////////////////////////////////////////////////
    while(!m_comm_serial.SerialIsOpen())
    {
      m_comm_serial.StdSerialInit();
      rate_s.sleep();
    }
    //wait vehicle online//////////////////////////////////////////////////////////////////////////////////////////////////
    while(!m_comm_transform.IsOnline())
    {
      m_comm_transform.DataRecvHandle(m_comm_serial.SerialDataRecv());
      ROS_ERROR("Vehicle offline");
      rate_s.sleep();
    }
    InfoVehicle info_vehicle_temp = m_comm_transform.GetVehicleInfo();
    switch(info_vehicle_temp.s_series)
    {
      case 1:
        ROS_INFO("Vehicle online : ECHO-STD");
        break;
      case 2:
        ROS_INFO("Vehicle online : ECHO-PRO");
        break;
      case 3:
        ROS_INFO("Vehicle online : ECHO-PLUS");
        break;
      case 4:
        ROS_INFO("Vehicle online : YORK-SLD");
        break;
      case 5:
        ROS_INFO("Vehicle online : YORK-MCNM");
        break;
      case 6:
        ROS_INFO("Vehicle online : YORK-AKM");
        break;
      default:
        break;
    }
    //enable device////////////////////////////////////////////////////////////////////////////////////////////////////////
    while(info_vehicle_temp.s_device_state == 0)
    {
      ROS_ERROR("Vehicle disable");
      m_comm_serial.SerialDataSend(m_comm_transform.StdVehicleSetState(1));
      rate_s.sleep();
      m_comm_transform.DataRecvHandle(m_comm_serial.SerialDataRecv());
      info_vehicle_temp = m_comm_transform.GetVehicleInfo();
    }
    //change to CAN mode///////////////////////////////////////////////////////////////////////////////////////////////////
    while(info_vehicle_temp.s_sys_mode != MODE_CAN)
    {
      ROS_ERROR("wrong mode");
      m_comm_serial.SerialDataSend(m_comm_transform.StdVehicleMode(MODE_CAN,BEEP_OFF));
      rate_s.sleep();
      m_comm_transform.DataRecvHandle(m_comm_serial.SerialDataRecv());
      info_vehicle_temp = m_comm_transform.GetVehicleInfo();
    }
    ROS_INFO("All clear,start to go");
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    current_time = ros::Time::now();
    last_time = current_time;
    while (ros::ok())
    {                        
        current_time = ros::Time::now();

        if(m_comm_serial.SerialIsOpen() == false)m_comm_serial.StdSerialInit();
        m_comm_transform.DataRecvHandle(m_comm_serial.SerialDataRecv());
        InfoVehicle info_vehicle = m_comm_transform.GetVehicleInfo();
        if(info_vehicle.s_device_state == 0)m_comm_serial.SerialDataSend(m_comm_transform.StdVehicleSetState(1));
        if(info_vehicle.s_sys_mode != MODE_CAN)m_comm_serial.SerialDataSend(m_comm_transform.StdVehicleMode(MODE_CAN,BEEP_OFF));

        //vehicle coordinate transform
        switch(info_vehicle_temp.s_series)
        {
          case 1: //ECHO_STD
          case 2: //ECHO_PRO
            m_vehcile_transform.TransformTWD((double)info_vehicle.s_odo_left/1000,(double)info_vehicle.s_odo_right/1000,(current_time - last_time).toSec(),0.24191);
            break;
          case 3: //ECHO_PLUS
            m_vehcile_transform.TransformTWD((double)info_vehicle.s_odo_left/1000,(double)info_vehicle.s_odo_right/1000,(current_time - last_time).toSec(),0.24191);
            break;
          case 4: //YORK-SLD
          case 5: //YORK-MCNM
            m_vehcile_transform.TransformBasic((double)info_vehicle.s_speed_x/1000,(double)info_vehicle.s_speed_y/1000,info_vehicle.s_rotate,(current_time - last_time).toSec());
            break;
          case 6: //YORK-AKM

            break;
          default:
            break;
        }

        //publish tf
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_vehcile_transform.m_msg_odom.th);
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = m_vehcile_transform.m_msg_odom.x;
        odom_tf.transform.translation.y = m_vehcile_transform.m_msg_odom.y;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_tf);
        //publish odom
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = m_vehcile_transform.m_msg_odom.x;
        odom.pose.pose.position.y = m_vehcile_transform.m_msg_odom.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = m_vehcile_transform.m_msg_odom.vx;
        odom.twist.twist.linear.y = m_vehcile_transform.m_msg_odom.vy;
        odom.twist.twist.angular.z = m_vehcile_transform.m_msg_odom.vth;
        pub_odom.publish(odom);

        last_time = current_time;
        ros::spinOnce();
        rate_ms.sleep();
    }
}
