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

bool following_wall = false;
bool crashed = false;

// Define the robot direction of movement
typedef enum _ROBOT_STATE {
    FIND_WALL = 0,
    TURN_LEFT,
    TURN_RIGHT,
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
    size_t range_size = laser_ranges.size();
    float left_side = 0.0, right_side = 0.0;
    float range_min = laser_msg.range_max, range_max = laser_msg.range_min;
    int nan_count = 0;
    for (size_t i = 0; i < range_size; i++) 
    {
        if (laser_ranges[i] < range_min) {
            range_min = laser_ranges[i];
        }

        if (std::isnan(laser_ranges[i])) {
            nan_count++;
        }
        if (i < range_size / 4) {
            if (laser_ranges[i] > range_max) {
                range_max = laser_ranges[i];
            }
        }

        if (i > range_size / 2) {
            left_side += laser_ranges[i];
        }
        else {
            right_side += laser_ranges[i];
        }
    }

    // Check if the robot has crashed into a wall
    if (nan_count > (range_size * 0.9) || laser_ranges[range_size / 2] < 0.25) {
        crashed = true;
    }
    else {
        crashed = false;
    }

    if (!crashed)
    {
        if (range_min <= 0.5) 
        {
            following_wall = true;
            crashed = false;

            if (left_side >= right_side) 
            {
                robot_state = TURN_RIGHT;
            }
            else {
                robot_state = TURN_LEFT;
            }
        }
        else
        {
            if (following_wall) 
            {
                if (range_max >= 2.0) {
                    following_wall = false;
                    robot_state = TURN_LEFT;
                }
            }
            if (!following_wall)
            {
                robot_state = FIND_WALL;
            }
            else
            {
                robot_state = FOLLOW_WALL;
            }
        }
    }
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

    // Initialize variables
    robot_state = FIND_WALL;
    motor_command.linear.x = 0.0;
    motor_command.angular.z = 0.0;

    // Enter an infinite loop where the laser_callback function will be called when new laser messages arrive
    ros::Rate loop_rate(20);
    while (ros::ok()) 
    {
        if (robot_state == FIND_WALL)
        {
            ROS_INFO("Current state: FIND_WALL");
            motor_command.linear.x = 0.2;
            motor_command.angular.z = -0.4;
        }
        else if (robot_state == TURN_LEFT)
        {
            ROS_INFO("Current state: TURN_LEFT");
            motor_command.linear.x = 0.0;
            motor_command.angular.z = 1.0;
        }
        else if (robot_state == TURN_RIGHT)
        {
            ROS_INFO("Current state: TURN_RIGHT");
            motor_command.linear.x = 0.0;
            motor_command.angular.z = -1.0;
        }
        else if (robot_state == FOLLOW_WALL)
        {
            ROS_INFO("Current state: FOLLOW_WALL");
            motor_command.linear.x = 0.4;
            motor_command.angular.z = 0.0;
        }
        else
        {
            ROS_INFO("ERROR STATE");
            motor_command.linear.x = 0.0;
            motor_command.angular.z = 0.0;
        }
        
        motor_command_publisher.publish(motor_command);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}