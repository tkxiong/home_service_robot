// ROS Libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // Motor Commands
#include "sensor_msgs/LaserScan.h" // Laser Data
#include "tf/transform_listener.h" // tf Tree

// C++ Libraries
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

// ROS Publisher:Motor Commands, Subscriber:Laser Data, and Messages:Laser Messages & Motor Messages
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;

// Define the robot direction of movement
typedef enum _ROBOT_STATE {
    FIND_WALL = 0,
    TURN_LEFT,
    FOLLOW_WALL
} ROBOT_STATE;

ROBOT_STATE robot_state;

// The laser_callback function will be called each time a laser scan data is received
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Read and process laser scan values
    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    float* laser_ranges_arr = &laser_ranges[0];
    size_t range_size = laser_ranges.size();
    float left_side = 0.0, right_side = 0.0;
    float range_min = laser_msg.range_max, range_max = laser_msg.range_min;

    float *result;
    result = std::min_element(laser_ranges_arr, laser_ranges_arr+299);
    float min_left_scan = *result;
    ROS_INFO("min_left_scan : %f", min_left_scan);

    result = std::min_element(laser_ranges_arr+300, laser_ranges_arr+339);
    float min_front_scan = *result;
    ROS_INFO("min_front_scan : %f", min_front_scan);

    result = std::min_element(laser_ranges_arr+340, laser_ranges_arr+639);
    float min_right_scan = *result;
    ROS_INFO("min_right_scan : %f", min_right_scan);
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "wall_follower_node");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 100
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);

    // Subscribe to the /scan topic and call the laser_callback function
    laser_subscriber = n.subscribe("/scan", 1000, laser_callback);

    robot_state = FIND_WALL;

    // Enter an infinite loop where the laser_callback function will be called when new laser messages arrive
    ros::Rate loop_rate(20);
    while (ros::ok()) 
    {
        // if (robot_state == FIND_WALL)
        // {
        //     ROS_INFO("Current state: FIND_WALL");
        //     motor_command.linear.x = 0.2;
        //     motor_command.angular.z = -0.3;
        // }
        // else if (robot_state == TURN_LEFT)
        // {
        //     ROS_INFO("Current state: TURN_LEFT");
        //     motor_command.linear.x = 0.0;
        //     motor_command.angular.z = 0.3;
        // }
        // else if (robot_state == FOLLOW_WALL)
        // {
        //     ROS_INFO("Current state: FOLLOW_WALL");
        //     motor_command.linear.x = 0.4;
        //     motor_command.angular.z = 0.0;
        // }
        // else
        // {
        //     ROS_INFO("ERROR STATE");
        //     motor_command.linear.x = 0.0;
        //     motor_command.angular.z = 0.0;
        // }
        
        // motor_command_publisher.publish(motor_command);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}