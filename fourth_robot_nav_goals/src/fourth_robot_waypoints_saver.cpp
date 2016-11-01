/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <ros/package.h>

#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>


using namespace visualization_msgs;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.20;
  marker.scale.y = msg.scale * 0.20;
  marker.scale.z = msg.scale * 0.20;
  marker.color.r = 0.05;
  marker.color.g = 0.80;
  marker.color.b = 0.02;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
    {
      mouse_point_ss << " at " << feedback->mouse_point.x
		     << ", " << feedback->mouse_point.y
		     << ", " << feedback->mouse_point.z
		     << " in frame " << feedback->header.frame_id;
    }

  switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
		       << "\nposition = "
		       << feedback->pose.position.x
		       << ", " << feedback->pose.position.y
		       << ", " << feedback->pose.position.z
		       << "\norientation = "
		       << feedback->pose.orientation.x
		       << ", " << feedback->pose.orientation.y
		       << ", " << feedback->pose.orientation.z
		       << ", " << feedback->pose.orientation.w
		       << "\nframe: " << feedback->header.frame_id
		       << " time: " << feedback->header.stamp.sec << "sec, "
		       << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
    }

  server->applyChanges();
}

void saveMarker( InteractiveMarker int_marker )
{
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}


void makeWayPointMarker(const tf::Vector3& position , const tf::Quaternion& orientation, const std::string &name)
{
  InteractiveMarker int_marker;

  int_marker.header.frame_id = "map";

  tf::pointTFToMsg(position, int_marker.pose.position);
  tf::quaternionTFToMsg(orientation, int_marker.pose.orientation);
  int_marker.scale = 1;

  int_marker.name = name.c_str();  
  int_marker.description = name.c_str();

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

 
  // we want to use our special callback function
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  //server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
  server->setCallback(int_marker.name, &processFeedback);
}


class GoalSaver
{
public:
  GoalSaver()
  {
    sub = nh.subscribe("clicked_point", 1, &GoalSaver::getGoalCB, this);

    // // create a timer to update the published transforms
    // ros::Timer frame_timer = nh.createTimer(ros::Duration(0.01), frameCallback);

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    ros::NodeHandle n("~");

    n.param<std::string>("savewaypointsfile", savefilename,
			 ros::package::getPath("fourth_robot_nav_goals") + "/waypoints/default.csv");
    ROS_INFO("[Save waypoints file name] : %s", savefilename.c_str());
    readExistGoals();

  }
  ~GoalSaver()
  {
	ROS_INFO("[Save waypoints file name] : %s", savefilename.c_str());
	save(savefilename.c_str());
	server.reset();
  }
  void readExistGoals()
  {
    const int rows_num = 7; // x, y, z, Qx, Qy, Qz, Qw
    boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
    std::ifstream ifs(savefilename.c_str());
    std::string line;
	
    while(ifs.good()){
      getline(ifs, line);
      if(line.empty()){ break; }
      tokenizer tokens(line, sep);
      std::vector<double> data;
      tokenizer::iterator it = tokens.begin();
      for(; it != tokens.end() ; ++it){
	std::stringstream ss;
	double d;
	ss << *it;
	ss >> d;
	data.push_back(d);
      }
      if(data.size() != rows_num){
	ROS_ERROR("Row size is mismatch!!");
	return;
      }else{
	position = tf::Vector3(data[0], data[1], 0.1);
	orientation = tf::Quaternion(data[3], data[4], data[5], data[6]);
	std::stringstream ss;
	ss << waypoints_name.size();
	std::string name = "WayPoint_" + ss.str();
	waypoints_name.push_back(name);
	makeWayPointMarker(position , orientation, waypoints_name.back());
        ROS_INFO("[%d]--> [X]: %f [Y]: %f", (int)waypoints_name.size(), data[0], data[1]);
      }
    }
    server->applyChanges();
    return;
  }
  void getGoalCB(const geometry_msgs::PointStamped &point)
  {
    position = tf::Vector3( point.point.x, point.point.y, 0.5);
    orientation = tf::createQuaternionFromRPY(0, 0, 0);
    std::stringstream ss;
    ss << waypoints_name.size();
    std::string name = "waypoint_" + ss.str();
    waypoints_name.push_back(name);
	
    makeWayPointMarker(position , orientation, waypoints_name.back());
    ROS_INFO_STREAM("\n" << point); 
    server->applyChanges();
  }

  void save(std::string filename)
  {
    std::ofstream savefile(filename.c_str(), std::ios::out);
    size_t size = waypoints_name.size();
    for(unsigned int i = 0; i < size; i++){
      server->get(waypoints_name[i], int_marker);
      //３次元の位置の指定
      savefile << int_marker.pose.position.x << "," 
	       << int_marker.pose.position.y << ","
	       << 0 << ","
	       << int_marker.pose.orientation.x << ","
	       << int_marker.pose.orientation.y << ","
	       << int_marker.pose.orientation.z << ","
	       << int_marker.pose.orientation.w << std::endl;
      //<< 1.0 << std::endl;
    }
  }
  void run(){
	ros::spin();
  }

private:
  int stasis;
  tf::Vector3 position;
  tf::Quaternion orientation;

  ros::NodeHandle nh;
  ros::Subscriber sub;

  InteractiveMarker int_marker;

  std::vector<std::string> waypoints_name;
  
  std::string savefilename;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  GoalSaver saver;
  saver.run();

}

