#ifndef VEHICLE_TRANSFORM_H
#define VEHICLE_TRANSFORM_H

#include <ros/ros.h>

/////////////////////////////////////////////////
struct  MsgOdom{
  double odom_r = 0.0;
  double odom_l = 0.0;
  double odom_b_r = 0.0;
  double odom_b_l = 0.0;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
};
//////////////////////////////////////////////////
class VehicleTransform
{
public:
  explicit VehicleTransform(void);
  ~VehicleTransform(void);

  void TransformBasic(double v_x,double v_y,double v_r,double dt);
  void TransformTWD(double odom_l,double odom_r,double dt,double wheel_dist);

public:
  MsgOdom m_msg_odom;

};

#endif // VEHICLE_TRANSFORM_H
