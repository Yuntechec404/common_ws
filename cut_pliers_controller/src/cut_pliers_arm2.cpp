#include <ros/ros.h>
#include <serial/serial.h>
#include <cut_pliers_controller/CmdCutPliers.h>
#include <sensor_msgs/Image.h>
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using std::string;

// 全局串口物件
serial::Serial ros_ser;
uint8_t FLAG_USART; //串口发送标志
uint8_t Flag_OK = 0;

// 全局變數（只控制手臂2）
int32_t S_H2;      // 手臂2目標高度
int32_t S_L2;      // 手臂2目標前伸長度
uint8_t S_En3;     // 手臂2使能狀態（電機3）
uint8_t S_En4;     // 手臂2使能狀態（電機4）
uint8_t S_C2;      // 手臂2爪子狀態

// 速度
float speed_h1 = 1;
float speed_l1 = 1;
float speed_h2 = 1;
float speed_l2 = 1;

// 用於接收下位機傳回的狀態
int32_t R_H2;      // 實際手臂2高度（單位：毫米）
int32_t R_L2;      // 實際手臂2長度（單位：毫米）
uint8_t R_En3;     // 實際使能狀態（電機3）
uint8_t R_En4;     // 實際使能狀態（電機4）
uint8_t R_C2;      // 實際爪子狀態
int32_t Voltage;   // 電池電壓

// 其他全局輔助變數
uint16_t count_1 = 0;
uint16_t a = 0, b = 0;
char aa;  // 用於儲存鍵盤輸入

// forward declaration
void send_data(void);
void receive_and_process_data(void);
void initialize_arms(void);

void chatterCallback(const geometry_msgs::Twist &msg)
{
    ROS_INFO("X_linear: [%g]", msg.linear.x);
    ROS_INFO("Y_linear: [%g]", msg.linear.y);
    ROS_INFO("Z_linear: [%g]", msg.linear.z);
    ROS_INFO("X_angular: [%g]", msg.angular.x);
    ROS_INFO("Y_angular: [%g]", msg.angular.y);
    ROS_INFO("Z_angular: [%g]", msg.angular.z);
    ROS_INFO("-------------");

    if(msg.linear.x>0 && msg.angular.z>0)
        FLAG_USART=1;
    else if(msg.linear.x>0 && msg.angular.z<0)
        FLAG_USART=0;
}

void send_data(void)
{
    uint8_t tbuf[31];
    tbuf[30] = 0;  // 校驗和初始化
    tbuf[0] = 0xAA;
    tbuf[1] = 0xAA;
    tbuf[2] = 0xF1;
    tbuf[3] = 34; // 數據長度
    for (int i = 4; i < 12; i++) tbuf[i] = 0; // arm1數據清零
    tbuf[12] = S_H2 & 0xFF;
    tbuf[13] = (S_H2 >> 8) & 0xFF;
    tbuf[14] = (S_H2 >> 16) & 0xFF;
    tbuf[15] = (S_H2 >> 24) & 0xFF;
    tbuf[16] = S_L2 & 0xFF;
    tbuf[17] = (S_L2 >> 8) & 0xFF;
    tbuf[18] = (S_L2 >> 16) & 0xFF;
    tbuf[19] = (S_L2 >> 24) & 0xFF;
    tbuf[20] = 0;
    tbuf[21] = 0;
    tbuf[22] = S_En3;
    tbuf[23] = S_En4;
    tbuf[24] = 0;
    tbuf[25] = S_C2;
    // 4個速度
    tbuf[30] = speed_h1;
    tbuf[31] = speed_l1;
    tbuf[32] = speed_h2;
    tbuf[33] = speed_l2;
    for (int i = 0; i < 34; i++) tbuf[34] += tbuf[i];
    try { ros_ser.write(tbuf, 34); }
    catch (serial::IOException &e) { ROS_ERROR("Unable to send data through serial port"); }
}

void receive_and_process_data(void)
{
    size_t n = ros_ser.available();
    a++;
    if(n>0)
    {
        uint8_t buffer[30], buf[30];
        if(n>=62){ while(n){n = ros_ser.available();if(n>=62)ros_ser.read(buf, 30);else {break;}} }
        if(n>=31 && n<62){ for(uint8_t i=0;i<n;i++){ if(buffer[0]!=0XAA)ros_ser.read(buffer, 1); else {break;} } }
        if(buffer[0]==0XAA)
        {
            ros_ser.read(buffer, 30);
            if(buffer[0]==0XAA && buffer[1]==0XF1)
            {
                uint8_t sum=0;
                for(uint8_t j=0;j<29;j++) sum+=buffer[j];
                if(buffer[29] == (uint8_t)(sum+buffer[0]))
                {
                    b++; Flag_OK=1;
                    R_H2 = (int32_t)((buffer[11]<<0)|(buffer[12]<<8)|(buffer[13]<<16)|(buffer[14]<<24));
                    R_L2 = (int32_t)((buffer[15]<<0)|(buffer[16]<<8)|(buffer[17]<<16)|(buffer[18]<<24));
                    R_En3 = buffer[21];
                    R_En4 = buffer[22];
                    R_C2  = buffer[24];
                    Voltage = (int32_t)((buffer[25]<<0)|(buffer[26]<<8)|(buffer[27]<<16)|(buffer[28]<<24));
                }
            }
            buffer[0]=0Xff;buffer[1]=0Xff;
        }
    }
    if(++count_1>149){
        count_1=0;
        std::cout<< "[01] Current_Height_2:" << (int)R_H2 <<"[mm]" << std::endl;
        std::cout<< "[02] Current_length_2:" << (int)R_L2 <<"[mm]" << std::endl;
        std::cout<< "[05] (Height_2)En3:" <<  (int)R_En3  << std::endl;
        std::cout<< "[06] (length_2)En4:" <<  (int)R_En4  << std::endl;
        std::cout<< "[09] State_Claw2:" <<   (int)R_C2  << std::endl;
        std::cout<< "[11] Voltage:" << (float)Voltage/100 << std::endl;
        std::cout<< "[12] 主循環頻數a:" << (uint16_t)a << std::endl;
        std::cout<< "[13] 有效接收數b:" << (uint16_t)b << std::endl;
        std::cout<< "[14] a/b:" <<  (float)a/b << std::endl;
        if(b>5000)b=b/10,a=a/10;
        std::cout<< "-----------------------" <<std::endl;
    }
}

void initialize_arms()
{
    S_H2 = 0;
    S_L2 = 0;
    S_En3 = 1;
    S_En4 = 1;
    S_C2 = 0;
    send_data();
}

class MinimalSubscriber
{
public:
    MinimalSubscriber(ros::NodeHandle& nh)
    { sub_ = nh.subscribe<sensor_msgs::Image>("Keyboard", 2, &MinimalSubscriber::callback, this); }
private:
    ros::Subscriber sub_;
    void callback(const sensor_msgs::Image::ConstPtr& msg) { aa = msg->height; }
};
class ArmStatusPublisher
{
public:
    ArmStatusPublisher(ros::NodeHandle& nh)
    {
        pub_ = nh.advertise<cut_pliers_controller::CmdCutPliers>("/arm_current_status", 1);
        timer_ = nh.createTimer(ros::Duration(0.1), &ArmStatusPublisher::timerCallback, this);
    }
private:
    ros::Publisher pub_;
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent&)
    {
        cut_pliers_controller::CmdCutPliers msg;
        msg.height2 = R_H2;
        msg.length2 = R_L2;
        msg.claw2 = (R_C2 != 0);
        msg.speed_h1 = speed_h1;
        msg.speed_l1 = speed_l1;
        msg.speed_h2 = speed_h2;
        msg.speed_l2 = speed_l2;
        pub_.publish(msg);
    }
};

class CmdCutPliersPublisher
{
public:
    CmdCutPliersPublisher(ros::NodeHandle& nh)
    {
        nh.param<std::string>("cmd_topic", cmd_topic_, "/cmd_cut_pliers_2");
        nh.param<int>("target_height", target_height_, 140);
        nh.param<int>("target_length", target_length_, -1);
        nh.param<int>("last_valid_length", last_valid_length_, 10);
        nh.param<bool>("claw", claw_, false);
        nh.param<bool>("allow_retract", allow_retract_, false);

        pub_ = nh.advertise<cut_pliers_controller::CmdCutPliers>(cmd_topic_, 10);
        sub_ = nh.subscribe(cmd_topic_, 10, &CmdCutPliersPublisher::cmdCutPliersCallback, this);
        timer_ = nh.createTimer(ros::Duration(0.1), &CmdCutPliersPublisher::timerCallback, this);

        ROS_INFO("Subscribed to topic: %s", cmd_topic_.c_str());
        ROS_INFO("target_height: %d, target_length: %d, last_valid_length: %d, claw: %d, allow_retract: %d",
                 target_height_, target_length_, last_valid_length_, (int)claw_, (int)allow_retract_);
    }
private:
    std::string cmd_topic_;
    int target_height_;
    int target_length_;
    int last_valid_length_;
    bool claw_;
    bool allow_retract_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;

    void cmdCutPliersCallback(const cut_pliers_controller::CmdCutPliers::ConstPtr& msg)
    {
        bool updated = false;
        if (msg->mode == 0)
        {
            if (!allow_retract_ && msg->length2 >= last_valid_length_)
            {
                if (target_length_ < msg->length2)
                {
                    target_length_ = msg->length2;
                    last_valid_length_ = target_length_;
                    updated = true;
                }
                else
                {
                    ROS_WARN("Ignoring forward command: target_length (%d) is less than current (%d)", msg->length2, last_valid_length_);
                }
            }
        }
        else if (msg->mode == 1)
        {
            if (msg->length2 < last_valid_length_)
            {
                target_length_ = msg->length2;
                last_valid_length_ = msg->length2;
                updated = true;
            }
            else
            {
                ROS_WARN("Ignoring reverse command: target_length (%d) is greater than current (%d)", msg->length2, last_valid_length_);
            }
        }
        if (msg->height2 >= 0 && target_height_ != msg->height2)
        {
            target_height_ = msg->height2;
            updated = true;
        }
        if (claw_ != msg->claw2)
        {
            claw_ = msg->claw2;
            updated = true;
        }
        if (updated)
        {
            ROS_INFO("Updated target: height2=%d, length2=%d, mode=%d", target_height_, target_length_, msg->mode);
        }
    }
    void timerCallback(const ros::TimerEvent&)
    {
        S_H2 = std::min(std::max(target_height_, 0), 280);
        S_L2 = std::min(std::max(last_valid_length_, 10), 440);
        S_C2 = claw_;
        send_data();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cut_pliers_controller_arm2");
    ros::NodeHandle nh("~");
    std::string port;
    int baudrate, timeout;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<int>("timeout", timeout, 100);
    ros_ser.setPort(port);
    ros_ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
    ros_ser.setTimeout(to);
    try { ros_ser.open(); }
    catch (serial::IOException& e) { ROS_ERROR("Unable to open serial port"); return -1; }
    if (ros_ser.isOpen())
        ROS_INFO("%s is opened.", port.c_str());
    else
        return -1;
    initialize_arms();
    MinimalSubscriber minimal_subscriber(nh);
    CmdCutPliersPublisher cmd_cut_pliers_publisher(nh);
    ArmStatusPublisher arm_status_publisher(nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate loop_rate(150);
    while (ros::ok()) {
        receive_and_process_data();
        loop_rate.sleep();
    }
    S_En3 = 0;
    S_En4 = 0;
    S_C2 = 0;
    send_data();
    ros_ser.close();
    ros::shutdown();
    return 0;
}
