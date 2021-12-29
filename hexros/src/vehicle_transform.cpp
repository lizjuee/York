#include "vehicle_transform.h"

VehicleTransform::VehicleTransform()
{

}
VehicleTransform::~VehicleTransform()
{

}
/*------------------------------------------------------------------------------------------------------------------
 * name: OdomCaculate
 * detail:
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleTransform::TransformBasic(double v_x,double v_y,double v_r,double dt)
{
  double delta_th = v_r * dt;
  double delta_x = (v_x * cos(delta_th) - v_y * sin(delta_th)) * dt;
  double delta_y = (v_x * sin(delta_th) + v_y * cos(delta_th)) * dt;

  m_msg_odom.x += cos(m_msg_odom.th) * delta_x - sin(m_msg_odom.th) * delta_y;
  m_msg_odom.y += sin(m_msg_odom.th) * delta_x + cos(m_msg_odom.th) * delta_y;
  m_msg_odom.th += delta_th;

  m_msg_odom.vx = v_x;
  m_msg_odom.vy = v_y;
  m_msg_odom.vth = v_r;
 //ROS_INFO("Basic convert: x=%fm y=%fm th=%f°/%frad",m_msg_odom.x,m_msg_odom.y,m_msg_odom.th/3.14*180,m_msg_odom.th);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: OdomBaseCaculate
 * detail:
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleTransform::TransformTWD(double odom_l,double odom_r,double dt,double wheel_dist)
{
  double d_odom_l =odom_l-m_msg_odom.odom_l;
  double d_odom_r =odom_r-m_msg_odom.odom_r;
  double d_avg = (d_odom_r + d_odom_l) / 2.0;
  double d_th = (d_odom_l - d_odom_r) / wheel_dist;
  m_msg_odom.vx = d_avg / dt;
  m_msg_odom.vy = 0;
  m_msg_odom.vth = d_th / dt;

  double delta_x = cos(d_th) * d_avg;
  double delta_y = -sin(d_th) * d_avg;

  m_msg_odom.odom_l = odom_l;
  m_msg_odom.odom_r= odom_r;
  m_msg_odom.x += cos(m_msg_odom.th) * delta_x - sin(m_msg_odom.th) * delta_y;
  m_msg_odom.y += sin(m_msg_odom.th) * delta_x + cos(m_msg_odom.th) * delta_y;
  m_msg_odom.th += d_th;
 //ROS_INFO("Basic convert: x=%fm y=%fm th=%f°/%frad",m_msg_odom.x,m_msg_odom.y,m_msg_odom.th/3.14*180,m_msg_odom.th);
}
