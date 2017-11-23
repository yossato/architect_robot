#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <sstream>
#include <fstream>
#include <numeric>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "architect_driver/architect_driver_node.hpp"

#include <sensor_msgs/JointState.h>

// for debug
#define printLOG( msg ) printf("%s,%d:%s\n",__FILE__,__LINE__,msg)

using namespace std;

ArchitectDriver::ArchitectDriver(ros::NodeHandle &n):
  geer_rate(2.0),
  delta_time(0),
  delta_dist_right(0),
  delta_dist_left(0),
  fd(-1),
  port_name("/dev/urbtc0"),
  ros_rate(100),
  tread(0.44515),
  wheel_diameter_right(0.38725),
  wheel_diameter_left(0.38695),
  max_linear_vel(1.1),
  max_angular_vel(M_PI),
  gain_p_right(1.0),
  gain_i_right(0),
  gain_p_left(1.0),
  gain_i_left(0),
  dead_zone_vel_right(0.05),
  dead_zone_vel_left(0.05),
  motor_pin_right(103),
  motor_pin_left(102),
  enc_pin_right(107),
  enc_pin_left(106),
  motor_pin_offset(101),
  enc_pin_offset(105),
  motor_pin_ch_right(1),
  motor_pin_ch_left(2)
{
  // ------ Get Params ------
  // robot property
  n.param("architect_driver/property/ros_rate", ros_rate, ros_rate);
  n.param("architect_driver/property/tread", tread, tread);
  n.param("architect_driver/property/wheel_diameter_right", wheel_diameter_right, wheel_diameter_right);
  n.param("architect_driver/property/wheel_diameter_left", wheel_diameter_left, wheel_diameter_left);
  // control
  n.param("architect_driver/control/max_linear_vel", max_linear_vel, max_linear_vel);
  n.param("architect_driver/control/max_angular_vel", max_angular_vel, max_angular_vel);
  n.param("architect_driver/control/gain_p_right", gain_p_right, gain_p_right);
  n.param("architect_driver/control/gain_i_right", gain_i_right, gain_i_right);
  n.param("architect_driver/control/gain_p_left", gain_p_left, gain_p_left);
  n.param("architect_driver/control/gain_i_left", gain_i_left, gain_i_left);
  n.param("architect_driver/control/dead_zone_vel_right", dead_zone_vel_right, dead_zone_vel_right);
  n.param("architect_driver/control/dead_zone_vel_left", dead_zone_vel_left, dead_zone_vel_left);
  // iMCs01
  n.param("architect_driver/imcs01/port_name", port_name, port_name);
  n.param("architect_driver/imcs01/motor_pin_right", motor_pin_right, motor_pin_right);
  n.param("architect_driver/imcs01/motor_pin_left", motor_pin_left, motor_pin_left);
  n.param("architect_driver/imcs01/enc_pin_right", enc_pin_right, enc_pin_right);
  n.param("architect_driver/imcs01/enc_pin_left", enc_pin_left, enc_pin_left);
  // ------ finish to get param ------

  // ------ prepare for motor control ------
  // 100 means max rpm of the motor unit (BLH230K-30)
  max_vel_right = wheel_diameter_right * M_PI * 100.0/60.0;
  max_vel_left = wheel_diameter_left * M_PI * 100.0/60.0;
  // velocity
  for(int i=0; i<2; i++){
	vel_right[i] = 0;
	vel_left[i] = 0;
  }
  // ------ finish to prepare for motor control ------
  
  // ------ iMCs01 ------
  // information
  // get order of pins
  motor_pin_right -= motor_pin_offset;
  motor_pin_left -= motor_pin_offset;
  enc_pin_right -= enc_pin_offset;
  enc_pin_left -= enc_pin_offset;
  // get channnel of pins
  motor_pin_ch_right = pow(2, motor_pin_right);
  motor_pin_ch_left = pow(2, motor_pin_left);
  // open serial
  if(openSerialPort() != 0){
	ROS_WARN("Could not connect to iMCS01.");
	ROS_BREAK();
  }
  // ------ end of iMCs01 ------

  // ------ Joint State Publisher ------
  js_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
  
}

ArchitectDriver::~ArchitectDriver()
{
  closeSerialPort();
}

int ArchitectDriver::openSerialPort()
{  
  // check if the iMCs01 is already open
  if(fd > 0)
	throw logic_error("imcs01 is already open");

  // open iMCs01
  fd = open(port_name.c_str(), O_RDWR);
  if(fd < 0)
    throw logic_error("Faild to open port: imcs01");

  // get oldtio
  tcgetattr(fd, &oldtio);
  
  //RETVAL
  cmd_ccmd.retval = 0;
  //offsetの初期化を行う
  cmd_ccmd.setoffset  = CH0 | CH1 | CH2 | CH3;
  //counterの初期化を行う
  cmd_ccmd.setcounter = CH0 | CH1 | CH2 | CH3;
  //resetint
  cmd_ccmd.resetint =  CH0 | CH1 | CH2 | CH3;				
  //全ピンエンコーダ(ソフトウェアカウンタ)
  cmd_ccmd.selin = SET_SELECT;
  //全ピンPWM出力
  cmd_ccmd.selout = SET_SELECT |  CH0 | CH1 | CH2 | CH3;
  //PWM出力を正にする。
  cmd_ccmd.posneg = SET_POSNEG | CH0 | CH1 | CH2 | CH3;
  
  for(int i=0; i<4; i++){
	//全 offset を0x7fffにする。PWM出力を0にすることを意味する。
	cmd_ccmd.offset[i] = 0x7fff;
	//全 counter を0にする。
	cmd_ccmd.counter[i] = 0;
  }

  //ブレーキをかけない(HIGH出力)
  cmd_ccmd.breaks = SET_BREAKS;   
  //magicnomber
  cmd_ccmd.magicno = 0x00;
  //wrrom
  cmd_ccmd.wrrom = 0;                                     

  // uin構造体cmd_ccmdの設定を書き込むためのioctl
  // (設定を変えるたびに呼び出す必要あり)
  if(ioctl(fd, URBTC_COUNTER_SET) < 0)
    throw logic_error("Faild to ioctl: URBTC_COUNTER_SET");
  //uin構造体cmd_ccmdの設定を書き込む
  if(write(fd, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){
	printLOG("Faild to write");
    throw logic_error("Faild to write");
  }

  if (ioctl(fd, URBTC_CONTINUOUS_READ) < 0){
  	printLOG("ioctl: URBTC_CONTINUOUS_READ error");
    throw logic_error("ioctl: URBTC_CONTINUOUS_READ error\n");
  }
  
  //counterの初期化を行わない
  cmd_ccmd.setcounter = 0;
  
  ROS_INFO("iMCs01 conneced to : %s", port_name.c_str());

  return 0;
}

int ArchitectDriver::closeSerialPort()
{
  cmd_ccmd.offset[motor_pin_right]  = 0x7fff;
  cmd_ccmd.offset[motor_pin_left]  = 0x7fff;
  cmd_ccmd.breaks = SET_BREAKS; 

  if(ioctl(fd, URBTC_COUNTER_SET) < 0)
    throw logic_error("Faild to ioctl: URBTC_COUNTER_SET");
  if(write(fd, &cmd_ccmd, sizeof(cmd_ccmd)) < 0)
    throw logic_error("Faild to write : iMCS01");

  cout << "[architect_driver] Stoped the motor." << endl;

  if(fd > 0){
    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    fd = -1;
  }
  
  cout << "[architect_driver] Closed the iMCs01 Port." << endl;
}

int ArchitectDriver::getEncoderData(ros::Time &time)
{  
  if(read(fd, &cmd_uin, sizeof(cmd_uin)) != sizeof(cmd_uin)){
	ROS_WARN("Failed to get Encoder info.");
	return 1;
  }
  else{
	time = ros::Time::now();
	return getEncoderCounts();
  }
}

int ArchitectDriver::getEncoderCounts()
{
  // ------ Rule ------
  //   current : 0
  //   last    : 1
  //   diff    : 2
  // ------------------  
  static double time[3] = {1, 0, 0};
  static int enc_cnt_right[3] = {0, 0, 0};
  static int enc_cnt_left[3] = {0, 0, 0};
  // set Joint State Info
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.name.push_back("right_wheel_joint");
  js.name.push_back("left_wheel_joint");
  // for rotation
  static double sum_rad_right = 0;
  static double sum_rad_left = 0;
  
  // ------ update current datas ------
  // get raw datas
  time[0] = (double)cmd_uin.time;
  enc_cnt_right[0] = (int)(cmd_uin.ct[enc_pin_right]);
  enc_cnt_left[0] = (int)(cmd_uin.ct[enc_pin_left]);    
  // Get diff time
  time[2] = time[0] - time[1];
  // Get diff of enc count
  enc_cnt_right[2] = enc_cnt_right[0] - enc_cnt_right[1];
  enc_cnt_left[2] = enc_cnt_left[0] - enc_cnt_left[1];
  
  // ------ check the overflow ------
  // about diff time
  if(time[2] < 0)
	  time[2] += 65536;     
  // about diff enc_cnt_right
  if(enc_cnt_right[2] > 6553)
	enc_cnt_right[2] = enc_cnt_right[2] - 65536;
  else if(enc_cnt_right[2] < -6553)
	enc_cnt_right[2] = enc_cnt_right[2] + 65536;
  // about diff enc_cnt_left
  if(enc_cnt_left[2] > 6553)
	enc_cnt_left[2] = enc_cnt_left[2] - 65536;
  else if(enc_cnt_left[2] < -6553)
	enc_cnt_left[2] = enc_cnt_left[2] + 65536;
  // ------ finish to check the over flow ------
  // ------ finish to update current datas ------

  // ------ update the output datas ------
  // get dist datas
  delta_dist_right = (enc_cnt_right[2]/4000.0/geer_rate)*(wheel_diameter_right*M_PI);
  delta_dist_left = -(enc_cnt_left[2]/4000.0/geer_rate)*(wheel_diameter_left*M_PI);
  // get delta time (change from [ms] to [s] on diff time)
  delta_time = time[2]/1000.0;
  if(delta_time <=0)
    delta_time = 1./ros_rate;

  // calcurate rotation
  sum_rad_right += enc_cnt_right[2]/4000.0*M_PI;
  sum_rad_left -= enc_cnt_left[2]/4000.0*M_PI;
  js.position.push_back(sum_rad_right);
  js.position.push_back(sum_rad_left);
  js_pub.publish(js);
  // ------ finish to update the output datas ------
  
  // ------ update the past datas ------
  time[1] = time[0];
  enc_cnt_right[1] = enc_cnt_right[0];
  enc_cnt_left[1] = enc_cnt_left[0];
  // ------ finish to update the past data ------
  
  return 0;
}

void ArchitectDriver::calculateOdometry(geometry_msgs::TransformStamped &odom_trans, nav_msgs::Odometry &odom)
{
  double delta_linear = (delta_dist_right + delta_dist_left)/2.0;
  double delta_yaw = (delta_dist_right - delta_dist_left)/tread;
  double approx_delta_linear = 0;
  double turn_rad = 0;
  static double x = 0;
  static double y = 0;
  static double yaw = 0;
  geometry_msgs::Quaternion odom_quat;
  
  // y = x と y = sin(x)の誤差が1%以下となる限界のx
  // つまり、 sin(x)=xと近似できる限界のxの値はx=0.245である。
  if(fabs(delta_yaw) < 245e-3){
	x += delta_linear * cos(yaw+(delta_yaw/2.0));
	y += delta_linear * sin(yaw+(delta_yaw/2.0));
	yaw += delta_yaw;
  } else{
	turn_rad = delta_linear/delta_yaw;
	approx_delta_linear = 2.0*turn_rad*sin((delta_yaw/2.0));
	x += approx_delta_linear * cos(yaw + (delta_yaw/2.0));
	y += approx_delta_linear * sin(yaw + (delta_yaw/2.0));
	yaw += delta_yaw;
  }  
  odom_quat = tf::createQuaternionMsgFromYaw(yaw);
  
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
}

void ArchitectDriver::culcurateVelocity(nav_msgs::Odometry &odom){
  // ------ Rule ------
  //   current : 0
  //   last    : 1
  // ------------------
  double linear_vel;
  double angular_vel;
 
  // ------ update current data ------
  // get raw data
  vel_right[0] = delta_dist_right / delta_time;
  vel_left[0] = delta_dist_left / delta_time;
  // ------ finish to update current data ------

  // ------ update output data ------
  // culculate linear and angular velocity
  linear_vel = (vel_right[0] + vel_left[0])/2.0;
  angular_vel = (vel_right[0] - vel_left[0])/tread;
  // input to odom data
  odom.twist.twist.linear.x = linear_vel;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = angular_vel;
  // ------ finish to update output data ------

  // ------ update past data ------
  vel_right[1] = vel_right[0];
  vel_left[1] = vel_left[0];
  // ------ finish to update past data ------
}

int ArchitectDriver::Drive(geometry_msgs::Twist cmd)
{
  double target_vel_right = 0;
  double target_vel_left = 0;
  bool brake = false;

  if(abs(cmd.linear.x) > abs(max_linear_vel))
	cmd.linear.x = cmd.linear.x/abs(cmd.linear.x) * max_linear_vel;
  if(abs(cmd.angular.z) > abs(max_angular_vel))
	cmd.angular.z = cmd.angular.z/abs(cmd.angular.z) * max_angular_vel;
  
  if(cmd.linear.x == 0 && cmd.angular.z == 0)
	brake = true;
	
  target_vel_right = cmd.linear.x + (tread/2.0)*cmd.angular.z;
  target_vel_left = cmd.linear.x - (tread/2.0)*cmd.angular.z;
  
  return driveDirect(target_vel_right, target_vel_left, brake);
}

int ArchitectDriver::driveDirect(double target_vel_right, double target_vel_left, bool brake)
{
  // for brake input
  unsigned char input_brake = SET_BREAKS;
  // for I-P control
  // ------ Rule ------
  //   current : 0
  //   last    : 1
  // ------------------
  static double control_input_vel_right[2] = {0, 0};
  static double control_input_vel_left[2] = {0, 0};
  static double error_vel_right[2] = {0, 0};
  static double error_vel_left[2] = {0, 0};
  static int over_vel_cnt_right = 0;
  static int over_vel_cnt_left = 0;
  static int cnt = 0;

  
  // =============================================================
  //                      I-P Control
  // 指令値の急激な変化にある程度鈍感な制御手法．I(積分)の項がある
  // ので，内部補償原理を満たす．
  // =============================================================
  // === Braking =========================================
  // 入力速度が 0 の場合，現在速度の逆を入力にすることで
  // I-Pでも即応性のあるブレーキングを試みている．
  // また，速度 0 付近での振動を抑制するために不感帯を
  // 設けている．
  //======================================================
  if(brake){
	target_vel_right = -vel_right[0];
	target_vel_left = -vel_left[0];
  }
  // ------ update current data ------  
  // error vel
  error_vel_right[0] = target_vel_right - vel_right[0];
  error_vel_left[0] = target_vel_left - vel_left[0];
  // control vel
  control_input_vel_right[0] = control_input_vel_right[1] + gain_p_right*(vel_right[0]-vel_right[1]) + (1.0/2.0)*gain_i_right*(error_vel_right[0]+error_vel_right[1]);
  control_input_vel_left[0] = control_input_vel_left[1] + gain_p_left*(vel_left[0]-vel_left[1]) + (1.0/2.0)*gain_i_left*(error_vel_left[0]+error_vel_left[1]);
  // ------ finish to update current data ------

  // ------ finish to update last data ------
  if(brake){
	if(abs(vel_right[0]) < dead_zone_vel_right && abs(vel_left[0]) < dead_zone_vel_left){
	  control_input_vel_right[0] = 0;
	  control_input_vel_left[0] = 0;
	  input_brake = input_brake | motor_pin_ch_right | motor_pin_ch_left;
	}
	else{
	  if(abs(vel_right[0]) < 0.05){
		control_input_vel_right[0] = 0;
		input_brake = input_brake | motor_pin_ch_right;
	  }
	  if(abs(vel_left[0]) < 0.05){
		control_input_vel_left[0] = 0;
		input_brake = input_brake | motor_pin_ch_left;
	  }
	}
  }

  if( (fabs(vel_right[0]-control_input_vel_right[0]) > 0.3) && fabs(vel_right[0]) < 0.25){
    ROS_WARN("Emergency Button Pushed...Right");
    control_input_vel_right[0] = 0;
  }
  if( (fabs(vel_left[0]-control_input_vel_left[0]) > 0.3) && fabs(vel_left[0]) < 0.25){
    ROS_WARN("Emergency Button Pushed...Left");
    control_input_vel_left[0] = 0;
  }  
  
  // ------ update last data ------
  // error vel
  error_vel_right[1] = error_vel_right[0];
  error_vel_left[1] = error_vel_left[0]; 
  // control vel
  control_input_vel_right[1] = control_input_vel_right[0];
  control_input_vel_left[1] = control_input_vel_left[0];
  // ==============================================================
	  
  // ------ write input datas to iMCs01 ------
  // culculate input data
  double input_rate_right = control_input_vel_right[0]/max_vel_right;
  double input_rate_left = control_input_vel_left[0]/max_vel_left;
  // ------ over flow ------
  if(abs(input_rate_right) > 1)
	input_rate_right /= abs(input_rate_right);
  if(abs(input_rate_left) > 1)
	input_rate_left /= abs(input_rate_left);
  // culculate input data
  cmd_ccmd.offset[motor_pin_right] =  (int)(0x7fff - 0x7fff*input_rate_right); 
  cmd_ccmd.offset[motor_pin_left] = (int)(0x7fff + 0x7fff*input_rate_left);
  //cout << cmd_ccmd.offset[motor_pin_right] << ", " << cmd_ccmd.offset[motor_pin_left] << endl;
  cmd_ccmd.breaks = input_brake;
  // write
  if(ioctl(fd, URBTC_COUNTER_SET) < 0){
	printLOG("Faild to ioctl: URBTC_COUNTER_SET");
	throw logic_error("Faild to ioctl: URBTC_COUNTER_SET");
  }
  if(write(fd, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){
	printLOG("Faild to write");
	throw logic_error("Faild to write");
  }
  // ------ finish to write input datas to iMCs01 ------
 
  return 0;
}

