/*
*******************************************
*/
#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>

using namespace std;
//创建一个serial类
serial::Serial sp;
bool debug = true;

#define to_rad 0.01745329f  //角度转弧度

uint8_t FLAG_USART; //串口发送标志
uint16_t count_2;//计数器

enum MotionState { FORWARD, BACKWARD, STOPPED, OTHER };
uint8_t Flag_start = 0;//下位机运行标志
float x_mid_speed;//
float z_mid_angle;//
float speed_A, speed_B, speed_C, speed_D;//发送到下位机的4个轮子的速度
int size;

uint16_t a,b;
void send_data(void);//串口发送协议函数
uint8_t Flag_OK=0;

// Global variables
double Yaw = 0.0;          // 內部IMU (rad)
double current_yaw = 0.0;  // 外部IMU (rad)
double target_yaw = 0.0;   // (rad)
double error = 0.0;

bool yaw_zero_set = false;
double yaw_zero = 0.0;

bool new_message_received = false;
string port = "/dev/ttyUSB0";
int rate;
string topic_odom, topic_imu, topic_cmd_vel;
bool publish_tf = true;

// 運動模型
float wheel_distance = 0.45;    // 左右輪距 (m)
float max_speed = 1.1;          // 最大輪速 (m/s)

// PID 參數
bool straight_correction = false;
bool external_imu = false;
double Kp = 0.035;// 比例增益
double Ki = 0.05;  // 積分增益
double Kd = 0.0; // 微分增益
double previous_error = 0.0; // 上一次誤差
double integral = 0.0;       // 誤差積分
double derivative = 0.0;     // 誤差微分

ros::Time current_time, last_time;
double dt;

/********** 定義結構 **********/

// 發送到下位機的數據結構
struct DownstreamData {
    float flag_start;    // 電機啟動標誌 (1.0: 啟動, 0.0: 停止)
    float speed_a;       // A 輪速度 (m/s)
    float speed_b;       // B 輪速度 (m/s)
    float speed_c;       // C 輪速度 (m/s)
    float speed_d;       // D 輪速度 (m/s)
    float reserved[7];   // 預留位
};

// 從下位機接收的數據結構
struct UpstreamData {
    float flag_start;    // 電機啟動標誌 (1.0: 啟動, 0.0: 停止)
    float gyro_roll;     // X 軸角速度 (原始數據)
    float gyro_pitch;    // Y 軸角速度 (原始數據)
    float gyro_yaw;      // Z 軸角速度 (原始數據)
    float accel_x;       // X 軸加速度 (原始數據)
    float accel_y;       // Y 軸加速度 (原始數據)
    float accel_z;       // Z 軸加速度 (原始數據)
    float yaw;           // Z 軸角度 (deg)
    float speed_a;       // A 輪速度 (m/s)
    float speed_b;       // B 輪速度 (m/s)
    float speed_c;       // C 輪速度 (m/s)
    float speed_d;       // D 輪速度 (m/s)
    float voltage;       // 電池電壓 (V * 100)
    float roll;          // X 軸角度 (deg)
    float pitch;         // Y 軸角度 (deg)
    float reserved[7];   // 預留位
};

DownstreamData Data_US;
UpstreamData Data_UR;

/*************************** */

/********** 運動控制 **********/
void compute_wheel_speeds(float vx, float vy, float omega, float& speed_a, float& speed_b, float& speed_c, float& speed_d) {
    // 假設車輛為四輪差分驅動，輪子分佈為矩形
    // A: 右前輪, B: 左前輪, C: 左後輪, D: 右後輪
    float L = wheel_distance / 2.0;

    // 運動學模型：輪速 = (線速度 ± 角速度 * 距離)
    speed_a = speed_d = vx + vy + omega * L; // 右前輪 // 右後輪
    speed_b = speed_c = vx - vy - omega * L; // 左前輪 // 左後輪

    // 限制最大速度
    speed_a = std::max(std::min(speed_a, max_speed), -max_speed);
    speed_b = std::max(std::min(speed_b, max_speed), -max_speed);
    speed_c = std::max(std::min(speed_c, max_speed), -max_speed);
    speed_d = std::max(std::min(speed_d, max_speed), -max_speed);
}
/*************************** */

/********** 回調函數 **********/
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  tf::Quaternion q(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w
  );
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw); // 轉換為歐拉角 (rad)
  current_yaw = yaw; // 弧度
}

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg) {//获取键盘控制的回调函数
    /* 四轮四驱 */  
    // ROS_INFO("X_linear: [%g]", msg.linear.x);
    // ROS_INFO("Y_linear: [%g]", msg.linear.y);
    // ROS_INFO("Z_linear: [%g]", msg.linear.z);
    // ROS_INFO("X_angular: [%g]", msg.angular.x);
    // ROS_INFO("Y_angular: [%g]", msg.angular.y);
    // ROS_INFO("Z_angular: [%g]", msg.angular.z);
    // ROS_INFO("-------------");
    static MotionState current_state = STOPPED;
    static MotionState previous_state = STOPPED;
    new_message_received = true;

    x_mid_speed = msg->linear.x;   // X 方向速度目標
    z_mid_angle = msg->angular.z;  // Z 軸旋轉速度目標

    // 限制最大速度 1.1 m/s
    x_mid_speed = std::max(std::min(x_mid_speed, max_speed), -max_speed);
    z_mid_angle = std::max(std::min(z_mid_angle, max_speed), -max_speed);

    // 判斷運動狀態
    if (z_mid_angle != 0) {
        current_state = OTHER;
    } else {
        if (x_mid_speed > 0) {
            current_state = FORWARD;
        } else if (x_mid_speed < 0) {
            current_state = BACKWARD;
        } else {
            current_state = STOPPED;
        }
    }

    // 運動狀態改變，重置直線校正
    if (current_state != previous_state) {
        // ROS_INFO("Motion state changed to %d", current_state);
        if (!external_imu)
            target_yaw = Yaw;
        else
            target_yaw = current_yaw;
        error = 0.0;
        previous_error = 0.0;
        integral = 0.0;
        derivative = 0.0;
    }

    // 計算輪速
    compute_wheel_speeds(x_mid_speed, 0.0, z_mid_angle, speed_A, speed_B, speed_C, speed_D);

    // 應用 PID 校正（僅直線運動）
    if (x_mid_speed!=0 && z_mid_angle==0 && straight_correction && dt > 0) { //按下 I 键 //按下 < 键 

        // 處理角度循環性 (-180° 到 180°)
        if (!external_imu)
            error = angles::normalize_angle(target_yaw - Yaw);
        else
            error = angles::normalize_angle(target_yaw - current_yaw);
        ROS_DEBUG("Straight Error:\t%.2f", error);

        integral += error * dt;
        derivative = (error - previous_error) / dt;
        double u = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;

        // 限制 u(t)
        if (u > max_speed || u < -max_speed) {
            ROS_ERROR("u(t) = %.2f overshoot", u);
            u = 0;
        }

        // 調整輪速（A、D 為右輪，B、C 為左輪）
        speed_A += u;
        speed_B -= u;
        speed_C -= u;
        speed_D += u;

        // 重新限制速度
        speed_A = std::max(std::min(speed_A, max_speed), -max_speed);
        speed_B = std::max(std::min(speed_B, max_speed), -max_speed);
        speed_C = std::max(std::min(speed_C, max_speed), -max_speed);
        speed_D = std::max(std::min(speed_D, max_speed), -max_speed);

    }
  
    // 設置電機啟動標誌
    if (x_mid_speed==0 && z_mid_angle==0) {
        speed_A = speed_B = speed_C = speed_D = 0;
        Flag_start = 0;
    } else {
        Flag_start = 1;
    }

    previous_state = current_state;
}

/*************************** */

typedef unsigned char byte;
float b2f(byte m0, byte m1, byte m2, byte m3) { //float 型解算为4个字节 
    if ((m0 == 0x00 || m0 == 0x80) && m1 == 0x00 && m2 == 0x00 && m3 == 0x00) return 0;

    //求符号位
    float sig = 1.;
        if (m0 >=128.)
        sig = -1.;

    //求阶码
    float jie = 0.;
    if (m0 >=128.) {
        jie = m0-128.;
    } else {
        jie = m0;
    }

    jie = jie * 2.;
    if (m1 >=128.)
        jie += 1.;
    jie -= 127.;

    //求尾码
    float tail = 0.;
    if (m1 >=128.)
        m1 -= 128.;
    tail =  m3 + (m2 + m1 * 256.) * 256.;
    tail  = (tail)/8388608;   //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1+tail);
    
    return f;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "listener");
    ros::NodeHandle np, private_np("~");//为这个进程的节点创建一个句柄
    ros::NodeHandle n;

    private_np.param<string>("port", port, "/dev/ttyUSB0");
    private_np.param<int>("rate", rate, 200);
    private_np.param<bool>("debug", debug, true);
    private_np.param<bool>("straight_correction", straight_correction, false);
    private_np.param<bool>("external_imu", external_imu, false);
    private_np.param<double>("Kp", Kp, 0.035);
    private_np.param<double>("Ki", Ki, 0.05);
    private_np.param<double>("Kd", Kd, 0.0);
    private_np.param<string>("topic_cmd_vel", topic_cmd_vel, "cmd_vel");
    private_np.param<string>("topic_imu", topic_imu, "/imu/data");
    private_np.param<string>("topic_odom", topic_odom, "odom");
    private_np.param<bool>("publish_tf", publish_tf, true);
    private_np.param<float>("wheel_distance", wheel_distance, 0.5);
    private_np.param<float>("max_speed", max_speed, 1.1);

    ros::Subscriber chatter_sub, imu_sub;
    chatter_sub = np.subscribe(topic_cmd_vel, 1000, chatterCallback);//订阅键盘控制
    if (external_imu)
        imu_sub = np.subscribe(topic_imu, 10, imuCallback);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(topic_odom, 50);
    ros::Publisher power_voltage_pub = n.advertise<std_msgs::Float64>("car_voltage", 10);
    std_msgs::Float64 voltage;

    tf::TransformBroadcaster odom_broadcaster;


    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;


    current_time = ros::Time::now();
    last_time = ros::Time::now();


    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort(port);
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try {
        //打开串口
        sp.open();
    } catch(serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if (sp.isOpen()) {
        ROS_INFO_STREAM(port << " is opened.");
    } else {
        return -1;
    }

    memset(&Data_US, 0, sizeof(DownstreamData));
    for(uint8_t j=0;j<3;j++) {
        send_data(); 
        ros::Duration(0.1).sleep(); //
    }

    ros::Rate loop_rate(rate);//设置循环间隔，即代码执行频率 200 HZ

    while (ros::ok()) {
        new_message_received = false;
        if (Flag_OK==1) {
            //角速度转换成 rad/s
            float angular_velocity_x = Data_UR.gyro_roll * 0.001064;
            float angular_velocity_y = Data_UR.gyro_pitch * 0.001064;
            float angular_velocity_z = Data_UR.gyro_yaw * 0.001064;
            //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
            float accelerated_speed_x = Data_UR.accel_x / 16384;
            float accelerated_speed_y = Data_UR.accel_y / 16384;
            float accelerated_speed_z = Data_UR.accel_z / 16384;
            
            
            //设定车子正负方向 ，车子前进方向为X正，后退为X负，左移为Y正，右移为Y负
            //轮子从上往下看，逆时针转为正角度，顺时针转为负角度
            float Power_A_X = +Data_UR.speed_a;
            float Power_A_Y = 0;
            float Power_B_X = -Data_UR.speed_b;
            float Power_B_Y = 0;
            float Power_C_X = -Data_UR.speed_c;
            float Power_C_Y = 0;
            float Power_D_X = +Data_UR.speed_d;
            float Power_D_Y = 0;

            vx  = (Power_A_X + Power_B_X + Power_C_X + Power_D_X)/4 ;//底盘当前X方向线速度 m/s	
            vy  = (Power_A_Y + Power_B_Y + Power_C_Y + Power_D_Y)/4 ;//底盘当前Y方向线速度 m/s	
            vth = angular_velocity_z;//设备当前Z轴角速度 rad/s

            //以给定机器人速度的典型方式计算里程计
            current_time = ros::Time::now();//记录当前时间
            dt = (current_time - last_time).toSec();

            if (!external_imu) {
                Yaw = Data_UR.yaw * to_rad;
                if (!yaw_zero_set) { // 設定偏航角零點
                    yaw_zero = Yaw;
                    yaw_zero_set = true;
                }
                th = angles::normalize_angle(Yaw - yaw_zero);
            } else {
                if (!yaw_zero_set) { // 設定偏航角零點
                    yaw_zero = current_yaw;
                    yaw_zero_set = true;
                }
                th = angles::normalize_angle(current_yaw - yaw_zero);
            }

            // delta_x, delta_y 使用 th
            double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
            double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            x += delta_x;//X轴速度累积位移 m
            y += delta_y;//Y轴速度累积位移 m

            //因为所有的里程表都是6自由度的，所以我们需要一个由偏航创建的四元数
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        
            //首先，我们将通过tf发布转换
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            //发送转换
            if (publish_tf) {
                odom_broadcaster.sendTransform(odom_trans);
            }


            //接下来，我们将通过ROS发布里程计信息
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            //设置位置
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = th;
            odom.pose.pose.orientation = odom_quat;

            //设定速度
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;

            // 設定共變異數矩陣
            odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3};
            odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                                        0, 1e-3, 0, 0, 0, 0,
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0,
                                        0, 0, 0, 0, 1e6, 0,
                                        0, 0, 0, 0, 0, 1e3};

            //发布消息
            odom_pub.publish(odom);

            last_time = current_time;//保存为上次时间
        }
		
        ros::spinOnce();

    // if (new_message_received) ROS_INFO("New /cmd_vel message received!");
    // else if (!new_message_received) ROS_WARN("No new /cmd_vel messages received.");

        if (new_message_received) {
            count_2=0;
                
            //若接收到键盘控制，则发送数据到下位机，同时接收下位机发送上来的数据		
            /*四轮四驱差速*/	
            /*<01>*/Data_US.flag_start = Flag_start ? 1.0f : 0.0f;//电机启动开关，1启动 0停止
            /*<02>*/Data_US.speed_a = speed_A; 
            /*<03>*/Data_US.speed_b = speed_B; 
            /*<04>*/Data_US.speed_c = speed_C; 
            /*<05>*/Data_US.speed_d = speed_D; //ABCD四轮的当前线速度 m/s
            for (int i = 0; i < 7; i++) {
                Data_US.reserved[i] = 0;
            }

            send_data(); //发送指令控制电机运行
        } else {  //當沒有收到/cmd_vel新的訊息以後，就停止運動
            count_2++;
            if (count_2 > 50) {
                Data_US.flag_start = Flag_start ? 1.0f : 0.0f;
                Data_US.speed_a = 0;
                Data_US.speed_b = 0;
                Data_US.speed_c = 0;
                Data_US.speed_d = 0;
                for (int i = 0; i < 7; i++) {
                    Data_US.reserved[i] = 0;
                }

                send_data();
            }
        }


        // 獲取下位機的數據
        size_t n = sp.available();//获取缓冲区内的字节数
        a++;
        if (n>0) {
            uint8_t buffer[64];uint8_t buf[64];

            if (n>=130) {
                while (n) {
                    n = sp.available();
                    if (n>=130) 
                        sp.read(buf, 62);
                    else {
                        break;
                    }
                }//砍掉旧缓存，获取最新数据 
            }
            if (n>=65 && n<130) {
                for (uint8_t i=0;i<n;i++) {
                    if(buffer[0]!=0XAA)
                        sp.read(buffer, 1);
                    else {break;}
                }//逐个读字节，读到帧头跳出 
            }
            if (buffer[0]==0XAA) { //
                sp.read(buffer, 64);//读出64个字节 
                if (buffer[0]==0XAA && buffer[1]==0XF1) {
                    uint8_t sum=0;
                    for (uint8_t j=0;j<63;j++)
                        sum+=buffer[j];    //计算校验和
                    if (buffer[63] == (uint8_t) sum+buffer[0]) {
                        b++;
                        Flag_OK=1;
                        // 解析數據到 UpstreamData
                        float* data_ptr = (float*)&Data_UR;
                        for (uint8_t i = 0; i < 15; i++) {
                            data_ptr[i] = b2f(buffer[4 * i + 3], buffer[4 * i + 4], buffer[4 * i + 5], buffer[4 * i + 6]);
                        }
                    }
                }
                buffer[0]=0Xff;
                buffer[1]=0Xff;
            }
        }

        /*<00>*///Data_UR[0] ;//电机启动开关，1启动 0停止
        /*<01>*///Data_UR[1] ;//X轴角速度原始数值 
        /*<02>*///Data_UR[2] ;//Y轴角速度原始数值 
        /*<03>*///Data_UR[3] ;//Z轴角速度原始数值
        /*<04>*///Data_UR[4] ;X轴加速度原始数值
        /*<05>*///Data_UR[5] ;Y轴加速度原始数值
        /*<06>*///Data_UR[6] ;Z轴加速度原始数值    
        /*<07>*///Data_UR[7] ;Z轴角度    
        /*<08>*///Data_UR[8] ;A轮速度
        /*<09>*///Data_UR[9] ;B轮速度 
        /*<10>*///Data_UR[10];C轮速度 
        /*<11>*///Data_UR[11];D轮速度
        /*<12>*///Data_UR[12];电压值*100 
        /*<13>*///Data_UR[13]; //预留
        /*<14>*///Data_UR[14]; //预留

        if (debug) {
            ROS_INFO("[00] Flag_start: %s", Data_UR.flag_start==1 ? "ON" : "OFF");
            ROS_INFO("[01] gyro_Roll: [%d]",  (int)Data_UR.gyro_roll);     //X轴角速度原始数据 gyro_roll
            ROS_INFO("[02] gyro_Pitch: [%d]", (int)Data_UR.gyro_pitch);    //Y轴角速度原始数据 gyro_pitch
            ROS_INFO("[03] gyro_Yaw: [%d]",   (int)Data_UR.gyro_yaw);      //Z轴角速度原始数据 gyro_yaw

            ROS_INFO("[04] accel_x: [%d]",  (int)Data_UR.accel_x); //X轴加速度原始数据 accel_x
            ROS_INFO("[05] accel_y: [%d]",  (int)Data_UR.accel_y); //Y轴加速度原始数据 accel_y
            ROS_INFO("[06] accel_z: [%d]",  (int)Data_UR.accel_z); //Z轴加速度原始数据 accel_z

            ROS_INFO("[07] Yaw: [%.2f deg]",  Data_UR.yaw); //绕Z轴角度 deg
                
            ROS_INFO("[08] Current_linear_A: [%.2f m/s]", +Data_UR.speed_a); //A轮线速度 m/s
            ROS_INFO("[09] Current_linear_B: [%.2f m/s]", -Data_UR.speed_b); //B轮线速度 m/s
            ROS_INFO("[10] Current_linear_C: [%.2f m/s]", -Data_UR.speed_c); //C轮线速度 m/s
            ROS_INFO("[11] Current_linear_D: [%.2f m/s]", +Data_UR.speed_d); //D轮线速度 m/s

            ROS_INFO("[12] Voltage: [%.2f V]", Data_UR.voltage / 100); // 电池电压
            ROS_INFO("[13] Roll: [%.2f deg]", Data_UR.roll);    //绕X轴角度 deg
            ROS_INFO("[14] Pitch: [%.2f deg]", Data_UR.pitch);  //绕Y轴角度 deg
            
            // ROS_INFO("a: [%d ]",   a);
            // ROS_INFO("b: [%d ]",   b);
            // ROS_INFO("a/b: [%.2f ]",   (float)a/(float)b);
            ROS_INFO("-----------------------");
        }

        if(b > 5000)
            b = b/10, a = a/10;

        
        voltage.data = (double) round(Data_UR.voltage) / 100;
        power_voltage_pub.publish(voltage);
        
        loop_rate.sleep();
    }
  

    memset(&Data_US, 0, sizeof(DownstreamData));
    for (uint8_t j=0;j<3;j++) {
        send_data();
        ros::Duration(0.1).sleep(); // 別移除
    }

    //关闭串口
    sp.close();

    return 0;
}


/************************ 發送12個數據 **************************/ 
void send_data(void) {
    uint8_t len = 12; // 數據字段數量
    uint8_t tbuf[53];

    // 將結構體視為連續的浮點數數組
    float* data_ptr = (float*)&Data_US;
    for (uint8_t i = 0; i < len; i++) {
        unsigned char* p = (unsigned char*)&data_ptr[i];
        tbuf[4 * i + 4] = *(p + 3); // 高位字節
        tbuf[4 * i + 5] = *(p + 2);
        tbuf[4 * i + 6] = *(p + 1);
        tbuf[4 * i + 7] = *(p + 0); // 低位字節
    }

    tbuf[len * 4 + 4] = 0;  // 校驗和初始值
    tbuf[0] = 0xAA;         // 幀頭
    tbuf[1] = 0xAA;         // 幀頭
    tbuf[2] = 0xF1;         // 功能字
    tbuf[3] = len * 4;      // 數據長度
    for (uint8_t i = 0; i < (len * 4 + 4); i++) {
        tbuf[len * 4 + 4] += tbuf[i]; // 計算校驗和
    }

    try {
        sp.write(tbuf, len * 4 + 5);
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to send data through serial port");
    }
}