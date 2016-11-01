/*-------------------------------------------------
参考プログラム
read_csv.cpp : https://gist.github.com/yoneken/5765597#file-read_csv-cpp

-------------------------------------------------- */


#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>

#include <ros/package.h>

// #include "sound/sound_service.h"
// #include "rospeex_if/rospeex.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

#define NAVIGATING		0
#define NEARGOAL		1
#define INITNAVI		2


double calculateDistance(double target_x, double target_y , double now_x, double now_y) 
{
  return  sqrt(pow((target_x - now_x), 2.0) + pow((target_y - now_y), 2.0));
}


class MyMoveBaseClient
{
public:
  MyMoveBaseClient() : ac("move_base", true), rate(100)
  {
	stasis = INITNAVI;
	target_num = 0;
	std::string filename;

	ros::NodeHandle n("~");
	n.param<std::string>("waypointsfile", filename, 
						 ros::package::getPath("fourth_robot_nav_goals") 
						 + "/waypoints/default.csv");
	n.param("start_target_num", target_num, 0);
	ROS_INFO("[Start target num] : %d", target_num);
	ROS_INFO("[Waypoints file name] : %s", filename.c_str());

    ROS_INFO("Reading Waypoints.");
	readWaypoint(filename.c_str());
   
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();

	// speeking
	// client = nh.serviceClient<sound::sound_service>("sound_service");
	// srv.request.situation = "start.wav";
	// client.call(srv);
  }

  void sendNewGoal()
  {
	move_base_msgs::MoveBaseGoal goal;
	goal = goals[target_num];
    // Need boost::bind to pass in the 'this' pointer
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal,
                boost::bind(&MyMoveBaseClient::doneCb, this, _1),
                boost::bind(&MyMoveBaseClient::activeCb, this),
                boost::bind(&MyMoveBaseClient::feedbackCb, this, _1));
	stasis = NAVIGATING;
  }

  void doneCb(const actionlib::SimpleClientGoalState& state)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
	stasis = INITNAVI;
  }

  // Called once when the goal becomes active
  void activeCb()
  {
	ROS_INFO("Goal just went active");
  }

  // Called every time feedback is received for the goal.
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
	//ROS_INFO("[X]: %f [Y]: %f", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y);
	double now_x = feedback->base_position.pose.position.x;
	double now_y = feedback->base_position.pose.position.y;
	double tar_x = goals[target_num-1].target_pose.pose.position.x;
	double tar_y = goals[target_num-1].target_pose.pose.position.y;
	double dist = calculateDistance(tar_x, tar_y, now_x, now_y);
	// ROS_INFO("[num ]: %d", target_num);
	// ROS_INFO("[tarX]: %lf [tarY]: %lf", tar_x, tar_y);
	// ROS_INFO("[nowX]: %lf [nowY]: %lf", now_x, now_y);
	if(dist <= 2.0){
	  stasis = NEARGOAL;
	  ROS_INFO("Reached WayPT[%d]!!! [Dist] : %lf",target_num, dist);
	}else{
	  ROS_INFO("Search WayPT[%d] [Dist]: %lf",target_num, dist);
	}
  }

  int getStasis(){
	return stasis;
  }

  void cancelGoal(){
	ROS_INFO("cancelGoal() is called !!");
	ac.cancelGoal();
  }

  int readWaypoint(std::string filename)
  {
    const int rows_num = 9; // x, y, z, Qx,Qy,Qz,Qw
    boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
    std::ifstream ifs(filename.c_str());
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
	return -1;
      }else{
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.pose.position.x    = data[0];
	goal.target_pose.pose.position.y    = data[1];
	goal.target_pose.pose.position.z    = data[2];
	goal.target_pose.pose.orientation.x = data[3];
	goal.target_pose.pose.orientation.y = data[4];
	goal.target_pose.pose.orientation.z = data[5];
	goal.target_pose.pose.orientation.w = data[6];

	goals.push_back(goal);
        ROS_INFO("[%d]--> [X]: %f [Y]: %f", (int)goals.size(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
      }
    }
    finish_num = (int)goals.size();
    return 0;
  }

  void speakWaypointsNumber(int num)
  {
	std::string waypointsname = "check";
	std::stringstream ss;
	ss << num;
	waypointsname = waypointsname + ss.str() + ".wav";
	// srv.request.situation = waypointsname.c_str();
	// client.call(srv);
  }

  void run()
  {
	int ret = 0;
	while(ros::ok()){
	  ret = stasis;
	  if(ret == NEARGOAL || ret == INITNAVI){
		if(ret == 1){
		  speakWaypointsNumber(target_num);
		  this->cancelGoal();
		}
		if(target_num != finish_num){
		  this->sendNewGoal();
		  target_num++;
		}else{
		  break; // Finish!
		}
	  }
	  ros::spinOnce();
	  rate.sleep();
	}// while(ros::ok())
  }

private:
  MoveBaseClient ac;
  int stasis;
  int target_num;
  int finish_num;
  ros::Rate rate;
  std::vector<move_base_msgs::MoveBaseGoal> goals;

  ros::NodeHandle nh;
  // ros::ServiceClient client;
  // sound::sound_service srv;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "fourth_robot_nav_send_goals");
  
  
  MyMoveBaseClient my_move_base_client;
 
  my_move_base_client.run();
 
  return 0;
}
