#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <iostream>

//------------------------------------------------------
// 全域變數與 serial 物件
//------------------------------------------------------
serial::Serial sp;

// 通訊參數
std::string port;
int baudrate;
int timeout_ms;

// 控制參數
std::string cmd_topic;
float target_height;
float target_length;

//------------------------------------------------------
// 回調
//------------------------------------------------------
void cmdCallback(const geometry_msgs::Twist& msg)
{
    ROS_INFO_STREAM("[CMD] linear.x=" << msg.linear.x
                    << " angular.z=" << msg.angular.z);

    // 根據需求判斷要不要送命令給下位機
    // 例如 msg.linear.x > 0 => 啟動動作
    //     msg.linear.x == 0 => 停止
    //     msg.angular.z => 控制方向 ...
    // 這裡你可以呼叫 send_data() 或自訂邏輯
}

//------------------------------------------------------
// 主程式
//------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "circular_saw");
    ros::NodeHandle nh("~");  // private namespace for params

    //------------------------------------------------------
    // 讀取參數
    //------------------------------------------------------
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<int>("timeout", timeout_ms, 100);
    nh.param<std::string>("cmd_topic", cmd_topic, "/circular_saw");
    nh.param<float>("target_height", target_height, 140.0f);
    nh.param<float>("target_length", target_length, -1.0f);

    ROS_INFO("=== Circular Saw Controller ===");
    ROS_INFO("Port: %s, Baud: %d, Timeout: %d", port.c_str(), baudrate, timeout_ms);
    ROS_INFO("cmd_topic: %s", cmd_topic.c_str());
    ROS_INFO("target_height=%.2f, target_length=%.2f", target_height, target_length);

    //------------------------------------------------------
    // 串口設定
    //------------------------------------------------------
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms);
    sp.setPort(port);
    sp.setBaudrate(baudrate);
    sp.setTimeout(to);

    try {
        sp.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port " << port);
        return -1;
    }

    if (sp.isOpen()) {
        ROS_INFO_STREAM(port << " is opened.");
    } else {
        return -1;
    }

    //------------------------------------------------------
    // Topic 訂閱
    //------------------------------------------------------
    ros::Subscriber sub = nh.subscribe(cmd_topic, 10, cmdCallback);

    //------------------------------------------------------
    // 主迴圈
    //------------------------------------------------------
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        // 這裡可以放你的 send_data()、read_data()、狀態列印等
        // if(FLAG_USART) send_data();
        // read_and_parse();

        loop_rate.sleep();
    }

    sp.close();
    return 0;
}
