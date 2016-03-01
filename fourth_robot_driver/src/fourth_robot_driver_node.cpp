#include <ros/ros.h>
#include <nav_msgs/Odometry.h>			
#include <geometry_msgs/Twist.h>		

#include <boost/thread.hpp>

#include "fourth_robot_driver/fourth_robot_driver_node.hpp"

boost::mutex access_mutex_;
geometry_msgs::Twist target_vel;

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  {
    boost::mutex::scoped_lock(access_mutex_);
	target_vel.linear.x = cmd_vel->linear.x;
	target_vel.linear.y = cmd_vel->linear.y;
	target_vel.linear.z = cmd_vel->linear.z;
	target_vel.angular.x = cmd_vel->angular.x;
	target_vel.angular.y = cmd_vel->angular.y;
	target_vel.angular.z = cmd_vel->angular.z;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fourth_robot_driver");
  ros::NodeHandle nh;
  FourthRobotDriver fourth_robot_driver(nh);

  // ------ set parameters ------
  string twist_sub_topic = "/cmd_vel";
  string odom_pub_topic = "/odom";
  string odom_parent_frame = "odom";
  string odom_child_frame = "base_footprint";
  double ros_rate;
  // parameter server
  nh.param("fourth_robot_driver/property/twist_sub_topic", twist_sub_topic, twist_sub_topic);
  nh.param("fourth_robot_driver/property/odom_pub_topic", odom_pub_topic, odom_pub_topic);
  nh.param("fourth_robot_driver/property/odom_parent_frame", odom_parent_frame, odom_parent_frame);
  nh.param("fourth_robot_driver/property/odom_child_frame", odom_child_frame, odom_child_frame);
  nh.param("fourth_robot_driver/property/ros_rate", ros_rate, ros_rate);
  // ------ finish to set parameters ------

  ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(twist_sub_topic.c_str(), 1, CmdVelCallback);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = odom_parent_frame.c_str();
  odom_trans.child_frame_id = odom_child_frame.c_str();
  tf::TransformBroadcaster odom_broadcaster;
  
  nav_msgs::Odometry odom;
  odom.header.frame_id = odom_parent_frame.c_str();
  odom.child_frame_id = odom_child_frame.c_str();
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_pub_topic.c_str(), 1);
  
  target_vel.linear.x = 0;
  target_vel.linear.y = 0;
  target_vel.linear.z = 0;
  target_vel.angular.x = 0;
  target_vel.angular.y = 0;
  target_vel.angular.z = 0;
  geometry_msgs::Twist local_target_vel;
  
  ros::Rate r(ros_rate);
  ros::Time time;
  
  ROS_INFO("... Fourth Robot setup finished ...");
  
  while(ros::ok()){	
	fourth_robot_driver.getEncoderData(time);
	odom_trans.header.stamp = time;        
	odom.header.stamp = time;
	
	fourth_robot_driver.calculateOdometry(odom_trans, odom);
	fourth_robot_driver.culcurateVelocity(odom);
	{
	  boost::mutex::scoped_lock(access_mutex_);
	  local_target_vel = target_vel;
	}
	fourth_robot_driver.Drive(local_target_vel);
	
	odom_broadcaster.sendTransform(odom_trans);
	odom_pub.publish(odom);

	ros::spinOnce();
	r.sleep();
  }
 
  return 0;
}
