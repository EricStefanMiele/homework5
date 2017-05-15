#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <homework5/MoveAction.h>
#include "move_base_msgs/MoveBaseActionGoal.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "move_client");

  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<homework5::MoveAction> ac("move", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  homework5::MoveGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.desired_speed = 50;
  goal.distance = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout){
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    homework5::MoveResultConstPtr ff = ac.getResult();
    nav_msgs::Odometry f = ff->odom_pose;
    ROS_INFO("Final odom --> x: %f - y: %f - z: %f", f.pose.pose.position.x, f.pose.pose.position.y, f.pose.pose.position.z);
  }
  else ROS_INFO("Action did not finish before the time out.");

  return 0;
}
