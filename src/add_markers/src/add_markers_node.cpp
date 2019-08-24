#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <pick_objects/goal.h>

ros::Publisher marker_pub;
pick_objects::goal current_goal;
bool incoming = false;
bool first_goal_clear = false;

void createBox(float duration, int id ,float x, float y)
{
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.25;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
}

void deleteBox(int id)
{
    visualization_msgs::Marker marker;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETE;
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;
    marker_pub.publish(marker);
}

void goalCallback(const pick_objects::goal::ConstPtr& msg)
{
    current_goal = *msg;
    createBox(0.0, 0, msg->first_x.data, msg->first_y.data);
    ROS_INFO("Pick up zone");
    incoming = true;

    if (msg->check_pos.data == false)
    {
        ROS_INFO("Demo");
        ROS_INFO("Loading object");
        ros::Duration(5.0).sleep(); // sleep for 5 second
        deleteBox(0);
        ros::Duration(5.0).sleep(); // sleep for 5 second
        createBox(0.0, 0, msg->second_x.data, msg->second_y.data);
        ROS_INFO("Unloading object");
        incoming = false;
    }
    
}

void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    if ((incoming == true) && (current_goal.check_pos.data == true))
    {
        if ((msg->status.status == 3) && (first_goal_clear == false))
        {
            first_goal_clear = true;
            ROS_INFO("Loading object");
            ros::Duration(5.0).sleep(); // sleep for 5 second
            deleteBox(0);
        }
        else if ((msg->status.status == 3) && (first_goal_clear == true))
        {
            ros::Duration(5.0).sleep(); // sleep for 5 second
            createBox(0.0, 0, current_goal.second_x.data, current_goal.second_y.data);
            ROS_INFO("Unloading object");
            first_goal_clear = false;
            incoming = false;
        }
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers_node");
    ros::NodeHandle n;

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber goal_sub = n.subscribe("pickup_goal", 1, goalCallback);
    ros::Subscriber result_sub = n.subscribe("move_base/result", 10, resultCallback);

    ros::spin();

    return 0;
}