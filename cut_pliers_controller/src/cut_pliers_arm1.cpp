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

// 全局變數（只控制手臂1）
int32_t S_H1;      // 手臂1目標高度
int32_t S_L1;      // 手臂1目標前伸長度
uint8_t S_En1;     // 手臂1使能狀態（電機1）
uint8_t S_En2;     // 手臂1使能狀態（電機2）
uint8_t S_C1;      // 手臂1爪子狀態

// 速度
float speed_h1 = 1;
float speed_l1 = 1;
float speed_h2 = 1;
float speed_l2 = 1;

// 用於接收下位機傳回的狀態
int32_t R_H1;      // 實際手臂1高度（單位：毫米）
int32_t R_L1;      // 實際手臂1長度（單位：毫米）
uint8_t R_En1;     // 實際使能狀態（電機1）
uint8_t R_En2;     // 實際使能狀態（電機2）
uint8_t R_C1;      // 實際爪子狀態
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
    // 封裝 S_H1 (4 bytes)
    tbuf[4]  = S_H1 & 0xFF;
    tbuf[5]  = (S_H1 >> 8) & 0xFF;
    tbuf[6]  = (S_H1 >> 16) & 0xFF;
    tbuf[7]  = (S_H1 >> 24) & 0xFF;
    // 封裝 S_L1 (4 bytes)
    tbuf[8]  = S_L1 & 0xFF;
    tbuf[9]  = (S_L1 >> 8) & 0xFF;
    tbuf[10] = (S_L1 >> 16) & 0xFF;
    tbuf[11] = (S_L1 >> 24) & 0xFF;
    // 手臂2相關數據置0
    for (int i = 12; i < 20; i++) tbuf[i] = 0;
    // 封裝使能狀態 S_En1, S_En2
    tbuf[20] = S_En1;
    tbuf[21] = S_En2;
    tbuf[22] = 0; // 使能3
    tbuf[23] = 0; // 使能4
    tbuf[24] = S_C1; // 爪子1
    tbuf[25] = 0;    // 爪子2
    // 4個速度
    tbuf[30] = speed_h1;
    tbuf[31] = speed_l1;
    tbuf[32] = speed_h2;
    tbuf[33] = speed_l2;
    // 校驗和
    for (int i = 0; i < 34; i++) tbuf[34] += tbuf[i];

    try { ros_ser.write(tbuf, 34); }
    catch (serial::IOException &e) {
        ROS_ERROR("Unable to send data through serial port");
    }
}

void receive_and_process_data(void)
{
    size_t n = ros_ser.available();
    a++;
    if(n>0)  
    {		   
        uint8_t buffer[30], buf[30];
        if(n>=62){
            while(n){n = ros_ser.available();if(n>=62)ros_ser.read(buf, 30);else {break;}}
        }
        if(n>=31 && n<62){
            for(uint8_t i=0;i<n;i++){
                if(buffer[0]!=0XAA)ros_ser.read(buffer, 1);
                else {break;}
            }
        }
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
                    R_H1  = (int32_t)((buffer[3]<<0)|(buffer[4]<<8)|(buffer[5]<<16)|(buffer[6]<<24));
                    R_L1  = (int32_t)((buffer[7]<<0)|(buffer[8]<<8)|(buffer[9]<<16)|(buffer[10]<<24));
                    R_En1 = buffer[19];
                    R_En2 = buffer[20];
                    R_C1  = buffer[23];
                    Voltage = (int32_t)((buffer[25]<<0)|(buffer[26]<<8)|(buffer[27]<<16)|(buffer[28]<<24));
                }
            }
            buffer[0]=0Xff;buffer[1]=0Xff;
        }
    }
    if(++count_1>149){
        count_1=0;
        std::cout<< "[01] Current_Height_1:" << (int)R_H1 <<"[mm]" << std::endl;
        std::cout<< "[02] Current_length_1:" << (int)R_L1 <<"[mm]" << std::endl;
        std::cout<< "[05] (Height_1)En1:" <<  (int)R_En1  << std::endl;
        std::cout<< "[06] (length_1)En2:" <<  (int)R_En2  << std::endl;
        std::cout<< "[09] State_Claw1:" <<   (int)R_C1  << std::endl;
        std::cout<< "[11] Voltage:" << (float)Voltage/100 << std::endl;
        std::cout<< "[12] 主循环频数a:" << (uint16_t)a << std::endl;
        std::cout<< "[13] 有效接收数b:" << (uint16_t)b << std::endl;
        std::cout<< "[14] a/b:" <<  (float)a/b << std::endl;
        if(b>5000)b=b/10,a=a/10;
        std::cout<< "-----------------------" <<std::endl;           
    }
}

void initialize_arms()
{
    S_H1 = 0;
    S_L1 = 0;
    S_En1 = 1;
    S_En2 = 1;
    S_C1 = 0;
    send_data();
}

//--- MinimalSubscriber/ArmStatusPublisher 無需修改（照舊） ---//
class MinimalSubscriber
{
public:
    MinimalSubscriber(ros::NodeHandle& nh)
    {
        sub_ = nh.subscribe<sensor_msgs::Image>("Keyboard", 2, &MinimalSubscriber::callback, this);
    }
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
        msg.height1 = R_H1;
        msg.length1 = R_L1;
        msg.claw1 = (R_C1 != 0);
        msg.speed_h1 = speed_h1;
        msg.speed_l1 = speed_l1;
        msg.speed_h2 = speed_h2;
        msg.speed_l2 = speed_l2;
        pub_.publish(msg);
    }
};

// -------- CmdCutPliersPublisher : 全部可 launch 設定 --------
class CmdCutPliersPublisher
{
public:
    CmdCutPliersPublisher(ros::NodeHandle& nh)
    {
        nh.param<std::string>("cmd_topic", cmd_topic_, "/cmd_cut_pliers_1");
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
        // 前進模式
        if (msg->mode == 0)
        {
            if (!allow_retract_ && msg->length1 >= last_valid_length_)
            {
                if (target_length_ < msg->length1)
                {
                    target_length_ = msg->length1;
                    last_valid_length_ = target_length_;
                    updated = true;
                }
                else
                {
                    ROS_WARN("Ignoring forward command: target_length (%d) is less than current (%d)", msg->length1, last_valid_length_);
                }
            }
        }
        // 後退模式
        else if (msg->mode == 1)
        {
            if (msg->length1 < last_valid_length_)
            {
                target_length_ = msg->length1;
                last_valid_length_ = msg->length1;
                updated = true;
            }
            else
            {
                ROS_WARN("Ignoring reverse command: target_length (%d) is greater than current (%d)", msg->length1, last_valid_length_);
            }
        }
        // 高度
        if (msg->height1 >= 0 && target_height_ != msg->height1)
        {
            target_height_ = msg->height1;
            updated = true;
        }
        // 爪子
        if (claw_ != msg->claw1)
        {
            claw_ = msg->claw1;
            updated = true;
        }
        if (updated)
        {
            ROS_INFO("Updated target: height=%d, length=%d, mode=%d", target_height_, target_length_, msg->mode);
        }
    }
    void timerCallback(const ros::TimerEvent&)
    {
        S_H1 = std::min(std::max(target_height_, 0), 280);
        S_L1 = std::min(std::max(last_valid_length_, 10), 440);
        S_C1 = claw_;
        send_data();
    }
};

// -----------------------------------------------------------------------------
// 主函數: 全部用 param 控制，便於 launch 靈活調用
// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cut_pliers_controller_arm1");
    ros::NodeHandle nh("~");

    // 讀取串口參數
    std::string port;
    int baudrate, timeout;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<int>("timeout", timeout, 100);

    ros_ser.setPort(port);
    ros_ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
    ros_ser.setTimeout(to);
    try {
        ros_ser.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR("Unable to open serial port");
        return -1;
    }
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

    S_En1 = 0;
    S_En2 = 0;
    S_C1 = 0;
    send_data();
    ros_ser.close();

    ros::shutdown();
    return 0;
}
