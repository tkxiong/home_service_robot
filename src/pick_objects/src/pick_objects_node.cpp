#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_objects/goal.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient *ac;

void goalCallback(const pick_objects::goal::ConstPtr& msg)
{
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = msg->first_x.data;
  goal.target_pose.pose.position.y = msg->first_y.data;
  goal.target_pose.pose.orientation.w = msg->first_w.data;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac->sendGoal(goal);

  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached its goal
  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Completed first goal");
  }
  else
  {
    ROS_INFO("first goal failed");
    return;
  }

  ros::Duration(5.0).sleep(); // sleep for 5 second

  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = msg->second_x.data;
  goal.target_pose.pose.position.y = msg->second_y.data;
  goal.target_pose.pose.orientation.w = msg->second_w.data;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac->sendGoal(goal);

  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached its goal
  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Completed second goal");
  }
  else
  {
    ROS_INFO("second goal failed");
  }
}

int main(int argc, char** argv)
{

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects_node");
  ros::NodeHandle n;

  ac = new MoveBaseClient("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Subscriber goal_sub = n.subscribe("pickup_goal", 1, goalCallback);

  ros::spin();

  return 0;
}