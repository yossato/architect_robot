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

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "fourth_robot_sound/sound_service.h"
#include "rospeex_if/rospeex.h"

#include <std_msgs/Int32.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

#define NAVIGATING		0
#define NEARGOAL		1
#define INITNAVI		2
#define HUMAN_DETECTED  3

double calculateDistance(double target_x, double target_y , double now_x, double now_y) 
{
  return  sqrt(pow(target_x - now_x, 2.0) + pow(target_y - now_y, 2.0));
}


class MyMoveBaseClient
{
public:
  void HmndtctReceived(const geometry_msgs::Point::ConstPtr& hmn_dtct);
public:
	MyMoveBaseClient() : ac("move_base", true), rate(100)
	{
		stasis = INITNAVI;
		target_num = 0;
		std::string filename;

		ros::NodeHandle n("~");
		n.param<std::string>("waypointsfile", filename, 
							 ros::package::getPath("third_robot_nav_goals") 
							 + "/waypoints/default.csv");
		n.param("start_target_num", target_num, 0);
		n.getParam("pause_waypoints", pause_waypoints_);

		ROS_INFO("[Start target num] : %d", target_num);

		num_of_pause_waypoints_ = checkPauseWaypoints(pause_waypoints_);
		ROS_INFO("[Num of pause waypoints] : %d", num_of_pause_waypoints_);

		ROS_INFO("[Waypoints file name] : %s", filename.c_str());

		ROS_INFO("Reading Waypoints.");
		if(readWaypoint(filename.c_str())){
			ROS_ERROR("Invalid waypoints file.");
		}

		// speeking
		ROS_INFO("Speak request: start");
		client = nh.serviceClient<third_robot_sound::sound_service>("third_robot_talker");
		// srv.request.situation = "start.wav";
		// client.call(srv);
   
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();

		// human detection
		hmn_dtct_sub = nh.subscribe<geometry_msgs::Point>("/hmn_dtct_test", 
														  1, 
														  boost::bind(&MyMoveBaseClient::HmndtctReceived, this, _1));
		waypoint_pub_ = nh.advertise<std_msgs::Int32>("/waypoints_number", 1);
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
		{
			boost::mutex::scoped_lock(stasis_mutex_);
			stasis = INITNAVI;
		}
	}

  // Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active");
	}

  // Called every time feedback is received for the goal.
	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
		double now_x = feedback->base_position.pose.position.x;
		double now_y = feedback->base_position.pose.position.y;
		double tar_x = goals[target_num-1].target_pose.pose.position.x;
		double tar_y = goals[target_num-1].target_pose.pose.position.y;
		double dist = calculateDistance(tar_x, tar_y, now_x, now_y);
		if(dist <= 3.0){
			{
				boost::mutex::scoped_lock(stasis_mutex_);
				stasis = NEARGOAL;
			}
			ROS_INFO("Reached !!! [Dist] : %lf", dist);
			ROS_INFO("WayPoint Number is : %d", target_num);
		}// else{
		//   ROS_INFO("[Dist]: %lf",dist);
		// }
	}


	bool checkDetectionArea(int number)
	{
		if((0 < number) && (number < detectcheck.size())){
			if(detectcheck[number] == 0){
				detectcheck[number] = 1;
				return true;
			}else{
				return false;
			}
		}
	}

	void cancelGoal(){
		//ROS_INFO("cancelGoal() is called !!");
		ac.cancelGoal();
	}

	int readWaypoint(std::string filename)
	{
		const int rows_num = 7; // x, y, z, w
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
				goal.target_pose.pose.position.x	= data[0];
				goal.target_pose.pose.position.y	= data[1];
				goal.target_pose.pose.position.z	= data[2];
				goal.target_pose.pose.orientation.x = data[3];
				goal.target_pose.pose.orientation.y = data[4];
				goal.target_pose.pose.orientation.z = data[5];
				goal.target_pose.pose.orientation.w = data[6];

				//		goal.target_pose.pose.orientation.w = data[3];
				goals.push_back(goal);
				ROS_INFO("[%d]--> [X]: %f [Y]: %f", (int)goals.size(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			}
		}
		finish_num = (int)goals.size();
		detectcheck.resize(goals.size());

		// detectcheck 初期化
		size_t size = detectcheck.size();
		for(size_t i = 0; i < size; i++){
			detectcheck[i] = 0;
		}

		return 0;
	}

	unsigned int checkPauseWaypoints(std::vector<int> &pause_waypoints)
	{
		unsigned int size = pause_waypoints.size();
		if(size <= 0)
		{
			return 0;
		}
		else
		{
			for(int i = 0; i < size; ++i)
			{
				if(pause_waypoints[i] < 0)
				{
					ROS_WARN("Invalid pause waypoints : [%d]", i);
				}
			}
			return size;
		}
	}

	void speakWaypointsNumber(int num)
	{
		std::string waypointsname = "check";
		std::stringstream ss;
		ss << num;
		waypointsname = waypointsname + ss.str() + ".wav";
		srv.request.situation = waypointsname.c_str();
		client.call(srv);
	}

	// reference :
	// http://answers.ros.org/question/63491/keyboard-key-pressed/
	int getch()
	{
		static struct termios oldt, newt;
		tcgetattr( STDIN_FILENO, &oldt);           // save old settings
		newt = oldt;
		newt.c_lflag &= ~(ICANON);                 // disable buffering      
		tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

		int c = getchar();  // read character (non-blocking)

		tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
		return c;
	}

	void isPauseWaypoints(int now_target)
	{
		if(num_of_pause_waypoints_ <= 0)
		{
			return;
		}
		else
		{
			if(pause_waypoints_[pause_waypoints_idx_] == target_num)
			{
				while(1)
				{
					ROS_INFO("Now Pause Waypoint at %d", target_num);
					ROS_INFO("Please Enter to restart !!!");
					int keyboard_input = getch();
					if(keyboard_input == '\n')
					{
						pause_waypoints_idx_++;
						return;
					}
				}
			}
		}		
	}

	void run()
	{
		int ret = 0;
		if(num_of_pause_waypoints_ > 0)
		{
			pause_waypoints_idx_ = 0;
		}
		while(ros::ok()){
			{
				boost::mutex::scoped_lock(stasis_mutex_);
				ret = stasis;
			}
			if(ret == NEARGOAL || ret == INITNAVI){
				if(ret == NEARGOAL){
					//speakWaypointsNumber(target_num);
					this->cancelGoal();
				}
				if(target_num != finish_num){
					this->isPauseWaypoints(target_num);
					this->sendNewGoal();
					target_num++;
				}else{
					srv.request.situation = "goal.wav";
					client.call(srv);
					break; // Finish!
				}
			}else if(ret == HUMAN_DETECTED){
				//if(checkDetectionArea(target_num)){ 
				this->cancelGoal(); // 一時停止
				srv.request.situation = "detect.wav";
				client.call(srv);
				this->sendNewGoal(); // 再スタート
				stasis = NAVIGATING;
				//}
			}

			std_msgs::Int32 now_waypoint;
			now_waypoint.data = target_num;
			waypoint_pub_.publish(now_waypoint);
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
	ros::ServiceClient client;
	third_robot_sound::sound_service srv;
	ros::Subscriber hmn_dtct_sub;
  ros::Publisher waypoint_pub_;
	boost::mutex stasis_mutex_;
	
	std::vector<int> detectcheck;

	std::vector<int> pause_waypoints_;
	unsigned int num_of_pause_waypoints_;
	int pause_waypoints_idx_;
	
	
};

void MyMoveBaseClient::HmndtctReceived(const geometry_msgs::Point::ConstPtr& hmn_dtct)
{
	ROS_INFO("Human or board detected!!!!");
	{
		boost::mutex::scoped_lock(access_mutex_);
		if(checkDetectionArea(target_num) == true){ 
			stasis = HUMAN_DETECTED;
		}
	}

}



int main(int argc, char** argv){
  ros::init(argc, argv, "third_robot_nav_send_goals_hmn_detect");
   
  MyMoveBaseClient my_move_base_client;
 
  my_move_base_client.run();
 
  return 0;
}
