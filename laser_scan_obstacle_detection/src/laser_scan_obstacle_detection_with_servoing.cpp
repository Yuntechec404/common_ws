// src/laser_scan_obstacle_detection_2xcamera.cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagActionGoal.h>
#include <string>
#include <unistd.h>
#define SQUARE_ROOT_2 1.414213562  // √2
using namespace std;

ros::Publisher cmd_vel_robot_pub;
geometry_msgs::Twist current_cmd_vel;
double navigation_stop_distance, servoing_stop_distance, servoing_cancel_obstacle_pause;
double zed_stop_distance, d435_stop_distance;
string pub_topic_name, tag_information_topic;
float front_min_distance = 5, servoing_min_distance;
bool msg_content = false, enable_servoing_obstacle_pause = true;

void ActionControl()
{
    geometry_msgs::Twist cmd_vel_robot;

    ROS_INFO("%f : %f", servoing_min_distance, d435_stop_distance);
    if (((current_cmd_vel.linear.x > 0 || current_cmd_vel.linear.z != 0) && front_min_distance <= zed_stop_distance) ||
        (current_cmd_vel.linear.x < 0 && servoing_min_distance <= d435_stop_distance && enable_servoing_obstacle_pause))
    {
        cmd_vel_robot.linear.x = 0.0;
        cmd_vel_robot.linear.y = 0.0;
        cmd_vel_robot.linear.z = 0.0;
        cmd_vel_robot.angular.x = 0.0;
        cmd_vel_robot.angular.y = 0.0;
        cmd_vel_robot.angular.z = 0.0;
        cmd_vel_robot_pub.publish(cmd_vel_robot);
        ROS_INFO("Closest obstacle");
        sleep(1);
    }
    else
    {
        cmd_vel_robot = current_cmd_vel;
        cmd_vel_robot_pub.publish(cmd_vel_robot);
    }
}

void FrontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float min_distance = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        if (std::isfinite(scan->ranges[i]) && scan->ranges[i] < min_distance)
        {
            min_distance = scan->ranges[i];
        }
    }
    front_min_distance = min_distance;
    // ROS_INFO("Closest obstacle distance: %f", zed_min_distance);
}

void ServoingCameraScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float min_distance = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        if (std::isfinite(scan->ranges[i]) && scan->ranges[i] < min_distance)
        {
            min_distance = scan->ranges[i];
        }
    }
    servoing_min_distance = min_distance;
    // ROS_INFO("Closest obstacle distance: %f", zed_min_distance);
}

void TagInformationCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    // ROS_INFO(" zxlkn zj");
    if(!msg->detections.empty())
    {
        float marker_distance;
        marker_distance = msg->detections[0].pose.pose.pose.position.z;
        if (marker_distance <= servoing_cancel_obstacle_pause)
            enable_servoing_obstacle_pause = false;
        // ROS_INFO("vuvohb  %d : %f", enable_servoing_obstacle_pause, marker_distance);
    }
}

void ApriltagServerGoalCallbake(const apriltag_ros::AprilTagActionGoal::ConstPtr& msg)
{
    // ROS_INFO("\n%d\n", msg->goal.goal);
    if (msg->goal.goal)
    {
        d435_stop_distance = servoing_stop_distance;
    }
    
    else if(!msg->goal.goal)
    {
        d435_stop_distance = navigation_stop_distance;
        enable_servoing_obstacle_pause = true;
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    msg_content = true;
    current_cmd_vel = *cmd_vel;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_obstacle_detection_with_servoing");
    ros::NodeHandle nh, private_nh("~");

    private_nh.param<double>("navigation_stop_distance", navigation_stop_distance, 0.5);
    private_nh.param<double>("servoing_stop_distance", servoing_stop_distance, 0.7);
    private_nh.param<double>("servoing_cancel_obstacle_pause", servoing_cancel_obstacle_pause, 0.7);
    private_nh.param<string>("publisher_topic_name", pub_topic_name, "cmd_vel_robot");
    private_nh.param<string>("tag_information_topic", tag_information_topic, "tag_detections");

    ros::Subscriber front_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("front_scan_filtered", 1000, FrontScanCallback);
    ros::Subscriber servoing_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("servoing_scan_filtered", 1000, ServoingCameraScanCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, cmdVelCallback);
    ros::Subscriber tag_information_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>(tag_information_topic, 1000, TagInformationCallback);
    ros::Subscriber apriltag_server_sub = nh.subscribe<apriltag_ros::AprilTagActionGoal>("/AprilTag_server/goal", 1000, ApriltagServerGoalCallbake);
    cmd_vel_robot_pub = nh.advertise<geometry_msgs::Twist>(pub_topic_name, 1000);

    zed_stop_distance = navigation_stop_distance*SQUARE_ROOT_2;
    d435_stop_distance = navigation_stop_distance;

    while(ros::ok())
    {
        msg_content = false;
        ros::spinOnce();
        if(msg_content)
        {
            ActionControl();
        }
    }
    return 0;
}
