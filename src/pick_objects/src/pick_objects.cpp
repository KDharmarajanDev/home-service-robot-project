#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Publisher reachedPickupPub = n.advertise<std_msgs::Bool>("reached_pickup",1);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickupGoal;

  // set up the frame parameters
  pickupGoal.target_pose.header.frame_id = "map";
  pickupGoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickupGoal.target_pose.pose.position.x = 1.0;
  pickupGoal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(pickupGoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Reached pickup location!");
    std_msgs::Bool boolean;
  	boolean.data = true;
    reachedPickupPub.publish(boolean);
  } else {
    ROS_INFO("Failed to reach pickup location!");
  }
  
  sleep(5);
  
  move_base_msgs::MoveBaseGoal dropoffGoal;

  // set up the frame parameters
  dropoffGoal.target_pose.header.frame_id = "map";
  dropoffGoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  dropoffGoal.target_pose.pose.position.x = 2.0;
  dropoffGoal.target_pose.pose.position.y = 2.0;
  dropoffGoal.target_pose.pose.position.z = 0;
  dropoffGoal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(dropoffGoal);

  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Reached the drop-off location");
  	std_msgs::Bool boolean;
  	boolean.data = false;
    reachedPickupPub.publish(boolean);
  } else {
    ROS_INFO("Failed to reach the drop-off location");
  }
  return 0;
}