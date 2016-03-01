#ifndef __FOURTH_ROBOT_DRIVER__
#define __FOURTH_ROBOT_DRIVER__

#include <termios.h>
#include <string>

// iMCs01
#include "iMCs01_driver/urbtc.h"
#include "iMCs01_driver/urobotc.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class FourthRobotDriver
{
public:
  FourthRobotDriver(ros::NodeHandle &n);
  ~FourthRobotDriver();

  int getEncoderData(ros::Time &time);
  void calculateOdometry(geometry_msgs::TransformStamped &odom_trans, nav_msgs::Odometry &odom);
  void culcurateVelocity(nav_msgs::Odometry &odom);
  int Drive(geometry_msgs::Twist cmd);
  
private:
  double ros_rate;
  string port_name;
  double wheel_base;
  double tread;
  double wheel_diameter_right;
  double wheel_diameter_left;
  double max_linear_vel;
  double max_angular_vel;
  double limit_vel_rate_right;
  double limit_vel_rate_left;
  double limit_vel_step_right;
  double limit_vel_step_left;
  double gain_p_right;
  double gain_i_right;
  double gain_p_left;
  double gain_i_left;
  int motor_pin_right;
  int motor_pin_left;
  int enc_pin_right;
  int enc_pin_left;
  int motor_pin_ch_right;
  int motor_pin_ch_left;
  double rc_filter_rate;
  
  int fd;
  termios oldtio;
  termios newtio;
  struct uin cmd_uin;
  struct ccmd cmd_ccmd;
  int motor_pin_offset;
  int enc_pin_offset;

  double max_enc_cnt;
  double geer_rate;
  double delta_time;
  double delta_dist_right;
  double delta_dist_left;
  double vel_right[2];
  double vel_left[2];
  double max_vel_right;
  double max_vel_left;

  // ------ function ------
  int openSerialPort();
  int closeSerialPort();
  int getEncoderCounts();
  int driveDirect(double target_vel_right, double target_vel_left, bool brake);
};
#endif 
