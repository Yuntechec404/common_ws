// src/laser_scan_obstacle_detection.cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <unistd.h>
using namespace std;

ros::Publisher cmd_vel_robot_pub;
geometry_msgs::Twist current_cmd_vel;
double stop_distance;
string pub_topic_name;
float zed_min_distance;
bool msg_content = false;

void ActionControl()
{
    // static bool enable = true;
    // static bool frequency = true;
    geometry_msgs::Twist cmd_vel_robot;

    if (zed_min_distance <= stop_distance)
    {
        cmd_vel_robot.linear.x = 0.0;
        cmd_vel_robot.linear.y = 0.0;
        cmd_vel_robot.linear.z = 0.0;
        cmd_vel_robot.angular.x = 0.0;
        cmd_vel_robot.angular.y = 0.0;
        cmd_vel_robot.angular.z = 0.0;
        // enable = false;
        ROS_INFO("Closest obstacle");
        sleep(1);
    }
    else
    {
        cmd_vel_robot = current_cmd_vel;
        // enable = true;
        // frequency = true;
    }

    cmd_vel_robot_pub.publish(cmd_vel_robot);

    // if(frequency)
    // {
    //     cmd_vel_robot_pub.publish(cmd_vel_robot);
    //     if(!enable) frequency = false;
    // }

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float min_distance = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        if (scan->ranges[i] < min_distance)
        {
            min_distance = scan->ranges[i];
        }
    }
    zed_min_distance = min_distance;
    // ROS_INFO("Closest obstacle distance: %f", zed_min_distance);

    // geometry_msgs::Twist cmd_vel_robot;
    // if (min_distance <= 0.5)
    // {
    //     cmd_vel_robot.linear.x = 0.0;
    //     cmd_vel_robot.linear.y = 0.0;
    //     cmd_vel_robot.linear.z = 0.0;
    //     cmd_vel_robot.angular.x = 0.0;
    //     cmd_vel_robot.angular.y = 0.0;
    //     cmd_vel_robot.angular.z = 0.0;
    // }
    // else
    // {
    //     cmd_vel_robot = current_cmd_vel;
    // }
    // cmd_vel_robot_pub.publish(cmd_vel_robot);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    msg_content = true;
    current_cmd_vel = *cmd_vel;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_obstacle_detection");
    ros::NodeHandle nh, private_nh("~");

    private_nh.param<double>("stop_distance", stop_distance, 0.5);
    private_nh.param<string>("publisher_topic_name", pub_topic_name, "cmd_vel_robot");

    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan_filtered", 1000, scanCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, cmdVelCallback);
    cmd_vel_robot_pub = nh.advertise<geometry_msgs::Twist>(pub_topic_name, 1000);

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
