#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include <string>

#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "circular_saw_controller/CircularSaw.h"

// 创建一个serial类
serial::Serial sp;

#define to_rad  0.017453f  // 角度转弧度

uint8_t FLAG_USART;        // 串口发送标志
uint16_t count_1, count_2; // 计数器

int size_;
int Voltage;

uint16_t a, b;
void send_data(void); // 串口发送协议函数

// 目標／指令相關變數
int32_t S_H1;
int32_t S_L1;

uint8_t S_En1;
uint8_t S_En2;
uint8_t S_En3;

int32_t R_H1;
int32_t R_L1;

uint8_t R_En1;
uint8_t R_En2;
uint8_t R_En3;

float Servo_Angle_RE;

float Servo_Angle_SE;
short Saw_speed_SE;

short Move_speed_L;
short Move_speed_H;

// 從參數設定的目標值（保留一份）
int32_t target_height_param = -100;
int32_t target_length_param = -100;
float   target_angle_param  = 0.0f;

// 是否輸出 log
bool enable_log = false;

// 串口與 topic 設定
std::string port_name = "/dev/ttyUSB0";
int baudrate = 115200;
int timeout_ms = 100;
std::string cmd_topic = "cmd_vel";

void cmdCallback(const circular_saw_controller::CircularSaw::ConstPtr& msg)
{
    if (msg->stop) {
        // 停機：送一筆「Saw_speed=0 + EN 全部關閉」的指令
        Saw_speed_SE = 0;
        Move_speed_H = 0;
        Move_speed_L = 0;

        // 位置維持在目前（不強制歸零），只關閉使能
        S_En1 = 0;
        S_En2 = 0;
        S_En3 = 0;

        FLAG_USART = 1;   // 讓 while 送出這一筆 STOP frame
        return;
    }

    // 一般 RUN 指令
    FLAG_USART = 1;

    S_L1          = msg->x_pos;     // X 位置 -> STM32 length1
    S_H1          = msg->z_pos;     // Z 位置 -> STM32 Height1
    Servo_Angle_SE= msg->angle;

    Move_speed_L  = msg->x_speed;
    Move_speed_H  = msg->z_speed;
    Saw_speed_SE  = msg->saw_speed;

    // RUN 狀態下啟用三軸
    S_En1 = 1;
    S_En2 = 1;
    S_En3 = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_saw");
    ros::NodeHandle nh;       // 一般 namespace
    ros::NodeHandle pnh("~"); // private namespace，用來取得 <param>

    //  讀取 launch 參數
    pnh.param<std::string>("port", port_name, std::string("/dev/ttyUSB1"));
    pnh.param<int>("baudrate", baudrate, 115200);
    pnh.param<int>("timeout", timeout_ms, 100);
    pnh.param<std::string>("cmd_topic", cmd_topic, std::string("/circular_saw_cmd"));
    pnh.param<int>("target_height", target_height_param, 0);
    pnh.param<int>("target_length", target_length_param, 0);
    pnh.param<float>("target_angle", target_angle_param, 0.0f);
    pnh.param<bool>("log", enable_log, false);

    ROS_INFO_STREAM("circular_saw config:");
    ROS_INFO_STREAM("port = " << port_name);
    ROS_INFO_STREAM("baudrate = " << baudrate);
    ROS_INFO_STREAM("timeout_ms = " << timeout_ms);
    ROS_INFO_STREAM("cmd_topic = " << cmd_topic);
    ROS_INFO_STREAM("target_height = " << target_height_param);
    ROS_INFO_STREAM("target_length = " << target_length_param);
    ROS_INFO_STREAM("target_angle = " << target_angle_param);
    ROS_INFO_STREAM("log = " << (enable_log ? "true" : "false"));

    ros::Subscriber sub = nh.subscribe(cmd_topic, 10, cmdCallback);

    // 创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms);
    // 设置要打开的串口名称
    sp.setPort(port_name);
    // 设置串口通信的波特率
    sp.setBaudrate(baudrate);
    // 串口设置timeout
    sp.setTimeout(to);

    try
    {
        // 打开串口
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port: " << port_name);
        return -1;
    }

    // 判断串口是否打开成功
    if (sp.isOpen())
    {
        ROS_INFO_STREAM(port_name << " is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(100); // 100 Hz
    
    S_H1 = target_height_param;
    S_L1 = target_length_param;
    Servo_Angle_SE= target_angle_param;
    Saw_speed_SE = 0;
    Move_speed_H = 2000;
    Move_speed_L = 3000;
    S_En1 = S_En2 = S_En3 = 1;

    while (ros::ok())
    {
        ros::spinOnce(); // 执行回调处理函数

        if (FLAG_USART == 1)
            send_data(); // 发送指令控制电机运行

        //============================
        //  連續讀取下位機資料
        //============================
        size_t n = sp.available(); // 获取缓冲区内的字节数
        a++;
        if (n > 0)
        {
            uint8_t buffer[24] = {0};
            uint8_t buf[24];

            if (n >= 48)
            {
                // 砍掉旧缓存，获取最新数据                  
                while (n)
                {
                    n = sp.available();
                    if (n >= 48)
                        sp.read(buf, 24);
                    else
                    {
                        break;
                    }
                }
            }

            if (n >= 24)
            {
                for (uint8_t i = 0; i < n; i++)
                {
                    if (buffer[0] != 0XAA)
                        sp.read(buffer, 1);
                    else
                    {
                        break;
                    }
                } // 逐个读字节，读到第一个帧头跳出
                n = sp.available();
            }

            if (buffer[0] == 0XAA && n >= 23)
            {
                sp.read(buffer, 23); // 读出剩余的23个字节
                if (buffer[0] == 0XAA && buffer[1] == 0XF1)
                {
                    uint8_t sum = 0;
                    for (uint8_t j = 0; j < 22; j++)
                        sum += buffer[j]; // 计算校验和

                    if (buffer[2] == 19 && buffer[22] == (uint8_t)(sum + 0XAA))
                    {
                        b++;
                        R_H1 = (int32_t)((buffer[3] << 0) | (buffer[4] << 8) | (buffer[5] << 16) | (buffer[6] << 24));  // 单位毫米
                        R_L1 = (int32_t)((buffer[7] << 0) | (buffer[8] << 8) | (buffer[9] << 16) | (buffer[10] << 24)); // 单位毫米

                        R_En1 = buffer[11];
                        R_En2 = buffer[12];
                        R_En3 = buffer[13];

                        Servo_Angle_RE = (float)((int32_t)((buffer[14] << 0) | (buffer[15] << 8) |
                                                           (buffer[16] << 16) | (buffer[17] << 24))) *
                                         0.01f;

                        Voltage = (int32_t)((buffer[18] << 0) | (buffer[19] << 8) |
                                            (buffer[20] << 16) | (buffer[21] << 24));
                    }
                }
                buffer[0] = 0Xff;
                buffer[1] = 0Xff;
            }
        }

        count_1++;
        if (enable_log && count_1 > 10)
        { // 显示频率降低为10HZ
            count_1 = 0;
            ROS_INFO("[01] Current_Height_1: [%d  mm]", R_H1);
            ROS_INFO("[02] Current_length_1: [%d  mm]", R_L1);
            ROS_INFO("[03] En1: [%d]", R_En1);
            ROS_INFO("[04] En2: [%d]", R_En2);
            ROS_INFO("[05] En3: [%d]", R_En3);
            ROS_INFO("[06] Servo_Angle_RE: [%.2f deg]", Servo_Angle_RE);
            ROS_INFO("[07] Voltage: [%.2f V]", (float)Voltage / 100); // 电池电压
            ROS_INFO("-----------------------");
            ROS_INFO("a: [%d ]", a);
            ROS_INFO("b: [%d ]", b);
            if (b != 0)
                ROS_INFO("a/b: [%.2f ]", (float)a / (float)b);
            if (b > 5000)
            {
                b = b / 10;
                a = a / 10;
            }
        }

        loop_rate.sleep(); // 循环延时时间
    }

    // 关闭串口
    sp.close();

    return 0;
}

//************************发送数据**************************// 

void send_data(void)
{
    uint8_t tbuf[26];

    tbuf[25] = 0;   // 校验位置零
    tbuf[0] = 0XAA; // 帧头
    tbuf[1] = 0XAA; // 帧头
    tbuf[2] = 0XF1; // 功能字
    tbuf[3] = 21;   // 数据长度

    tbuf[4]  = S_H1 >> 0;
    tbuf[5]  = S_H1 >> 8;
    tbuf[6]  = S_H1 >> 16;
    tbuf[7]  = S_H1 >> 24;

    tbuf[8]  = S_L1 >> 0;
    tbuf[9]  = S_L1 >> 8;
    tbuf[10] = S_L1 >> 16;
    tbuf[11] = S_L1 >> 24;

    tbuf[12] = (int)(Servo_Angle_SE * 100) >> 0;
    tbuf[13] = (int)(Servo_Angle_SE * 100) >> 8;
    tbuf[14] = (int)(Servo_Angle_SE * 100) >> 16;
    tbuf[15] = (int)(Servo_Angle_SE * 100) >> 24;

    tbuf[16] = Saw_speed_SE >> 0;
    tbuf[17] = Saw_speed_SE >> 8;

    tbuf[18] = Move_speed_H >> 0;
    tbuf[19] = Move_speed_H >> 8;
    tbuf[20] = Move_speed_L >> 0;
    tbuf[21] = Move_speed_L >> 8;

    tbuf[22] = S_En1;
    tbuf[23] = S_En2;
    tbuf[24] = S_En3;

    for (uint8_t i = 0; i < 25; i++)
        tbuf[25] += tbuf[i]; // 计算校验和

    try
    {
        sp.write(tbuf, 26); // 发送数据下位机(数组，字节数)
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to send data through serial port");
    }
}
