#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <homework5/MoveAction.h>

class MoveAction{

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<homework5::MoveAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  homework5::MoveFeedback feedback_;
  homework5::MoveResult result_;

  ros::Subscriber sub;
  bool success;
  bool odom_set;
  bool preempted;
  float desired_speed;
  float distance;
  float x;
  float y;
  float z;

public:

  MoveAction(std::string name) :
    as_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1), false),
    action_name_(name){
		success = false;
		odom_set = false;
		distance = -1.0;
		sub = nh_.subscribe("odom", 10, &MoveAction::chatterCallback, this);
		as_.start();
	}

  ~MoveAction(void){}

  void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
		float actual_x = msg->pose.pose.position.x;
		float actual_y = msg->pose.pose.position.y;
		float actual_z = msg->pose.pose.position.z;
		if(!odom_set) {
			x = actual_x;
			y = actual_y;
			z = actual_z;
			odom_set = true;
		}
		else if(distance >= 0){
			//ROS_INFO("euc dist: %f - dist: %f", sqrt(pow(actual_x-x,2) + pow(actual_y-y,2) + pow(actual_z-z,2)), distance);
			if(sqrt(pow(actual_x-x,2) + pow(actual_y-y,2) + pow(actual_z-z,2)) >= distance) {
				success = true;
				result_.odom_pose = *msg;
			}
		}
	}

  void executeCB(const homework5::MoveGoalConstPtr &goal){

		distance = goal->distance;
		desired_speed = goal->desired_speed;

		geometry_msgs::Twist to_cmd_vel;

		ros::Publisher pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // start executing the action
    while(!success){

			//ros::spin();
		  to_cmd_vel.linear.x = desired_speed;

		  pub.publish(to_cmd_vel);

	      // check that preempt has not been requested by the client
	      if (as_.isPreemptRequested() || !ros::ok()){
	        ROS_INFO("%s: Preempted", action_name_.c_str());
	        // set the action state to preempted
	        as_.setPreempted();
	        success = false;
	        break;
	      }
    }

    if(success){
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
		success = false;
		odom_set = false;
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "move_server");

  MoveAction move("move");
  ros::spin();

  return 0;
}
