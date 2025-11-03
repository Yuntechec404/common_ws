#include <ros/ros.h>
#include <serial/serial.h>
#include <cut_pliers_controller/CmdCutPliers.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <vector>
#include <array>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;
using std::string;

// ===================== 全局串口物件/旗標 =====================
serial::Serial ros_ser;
uint8_t FLAG_USART; // 串口發送標誌
uint8_t Flag_OK = 0;

// ===================== 下發(只控制手臂1) =====================
int32_t S_H1;      // 手臂1目標高度 (mm)
int32_t S_L1;      // 手臂1目標前伸長度 (mm)
uint8_t S_En1;     // 使能1
uint8_t S_En2;     // 使能2
uint8_t S_C1;      // 爪子1 (0/1)

// 速度（只轉發到 /arm_current_status，實際協議未使用）
float speed_h1 = 1;
float speed_l1 = 1;
float speed_h2 = 1;
float speed_l2 = 1;

// ===================== 回傳狀態 =====================
int32_t R_H1 = 0;  // 實際手臂1高度 (mm)
int32_t R_L1 = 0;  // 實際手臂1長度 (mm)
uint8_t R_En1 = 0; // 使能1
uint8_t R_En2 = 0; // 使能2
uint8_t R_C1  = 0; // 爪子1
int32_t Voltage = 0;

// ===================== 其他全域 =====================
uint16_t count_1 = 0;
uint16_t a = 0, b = 0;
char aa;  // 鍵盤輸入(沿用)

// RX 狀態監控
ros::Time last_rx_time;
static std::vector<uint8_t> rxbuf;

// ============== 前置宣告 ==============
void send_data(void);
void receive_and_process_data(bool log);
void initialize_arms(void);

// 小工具：節流 hexdump（印前 16 bytes）
static void dump_bytes_throttled(const std::vector<uint8_t>& v, const char* tag, double sec=1.0) {
    static ros::Time last;
    if ((ros::Time::now() - last).toSec() < sec) return;
    last = ros::Time::now();
    std::ostringstream oss;
    size_t n = std::min<size_t>(v.size(), 16);
    for (size_t i=0;i<n;i++) {
        oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << (int)v[i] << " ";
    }
    // ROS_WARN_STREAM(tag << " (" << v.size() << " bytes): " << oss.str());
}

// ============== 範例的 Twist callback（保留） ==============
void chatterCallback(const geometry_msgs::Twist &msg)
{
    ROS_INFO("X_linear: [%g]", msg.linear.x);
    ROS_INFO("Y_linear: [%g]", msg.linear.y);
    ROS_INFO("Z_linear: [%g]", msg.linear.z);
    ROS_INFO("X_angular: [%g]", msg.angular.x);
    ROS_INFO("Y_angular: [%g]", msg.angular.y);
    ROS_INFO("Z_angular: [%g]", msg.angular.z);
    ROS_INFO("-------------");

    if(msg.linear.x>0 && msg.angular.z>0) FLAG_USART=1;
    else if(msg.linear.x>0 && msg.angular.z<0) FLAG_USART=0;
}

// ============== 下發封包 ==============
void send_data(void)
{
    uint8_t tbuf[27] = {0};

    tbuf[0] = 0xAA;
    tbuf[1] = 0xAA;
    tbuf[2] = 0xF1;
    tbuf[3] = 22; // payload bytes count (from tbuf[4] to tbuf[25])

    // S_H1
    tbuf[4]  = (uint8_t)(S_H1 & 0xFF);
    tbuf[5]  = (uint8_t)((S_H1 >> 8) & 0xFF);
    tbuf[6]  = (uint8_t)((S_H1 >> 16) & 0xFF);
    tbuf[7]  = (uint8_t)((S_H1 >> 24) & 0xFF);

    // S_L1
    tbuf[8]  = (uint8_t)(S_L1 & 0xFF);
    tbuf[9]  = (uint8_t)((S_L1 >> 8) & 0xFF);
    tbuf[10] = (uint8_t)((S_L1 >> 16) & 0xFF);
    tbuf[11] = (uint8_t)((S_L1 >> 24) & 0xFF);

    // Arm2 reserved
    for (int i = 12; i < 20; ++i) tbuf[i] = 0;

    // Enables
    tbuf[20] = S_En1;
    tbuf[21] = S_En2;
    tbuf[22] = 0; // En3
    tbuf[23] = 0; // En4

    // Claws
    tbuf[24] = S_C1;
    tbuf[25] = 0;

    // checksum over [0..25]
    uint8_t sum = 0;
    for (int i = 0; i <= 25; ++i) sum += tbuf[i];
    tbuf[26] = sum;

    try { ros_ser.write(tbuf, sizeof(tbuf)); }
    catch (serial::IOException&) {
        ROS_ERROR("Unable to send data through serial port");
    }
}

// ============== 環形緩衝解析 + 偵錯 ==============
void receive_and_process_data(bool log)
{
    // 把所有可讀 bytes 先塞進 rxbuf
    size_t n = ros_ser.available();
    if (n > 0) {
        std::vector<uint8_t> tmp(n);
        try { ros_ser.read(tmp.data(), tmp.size()); }
        catch (serial::IOException&) { ROS_ERROR_THROTTLE(1.0, "serial read error"); tmp.clear(); }
        rxbuf.insert(rxbuf.end(), tmp.begin(), tmp.end());
    }

    const size_t FRAME_SZ = 27; // AA AA F1 | LEN=22 | payload(22) | CHK
    bool parsed_any = false;

    while (rxbuf.size() >= FRAME_SZ) {
        // 找 AA AA
        size_t p = 0;
        while (p+2 < rxbuf.size() && !(rxbuf[p]==0xAA && rxbuf[p+1]==0xAA)) ++p;
        if (p+2 >= rxbuf.size()) { rxbuf.erase(rxbuf.begin(), rxbuf.begin()+p); break; }

        // 剩下不足一幀
        if (rxbuf.size() - p < FRAME_SZ) {
            if (p > 0) rxbuf.erase(rxbuf.begin(), rxbuf.begin()+p);
            break;
        }

        // 功能字
        if (rxbuf[p+2] != 0xF1) {
            dump_bytes_throttled(rxbuf, "[RX] AA AA but func != F1, drop one byte");
            rxbuf.erase(rxbuf.begin()+p);
            continue;
        }

        // LEN
        uint8_t len = rxbuf[p+3];
        if (len != 22) {
            dump_bytes_throttled(rxbuf, "[RX] LEN != 22, drop AA");
            rxbuf.erase(rxbuf.begin()+p);
            continue;
        }

        // 取出一幀
        std::array<uint8_t, FRAME_SZ> fr{};
        std::copy(rxbuf.begin()+p, rxbuf.begin()+p+FRAME_SZ, fr.begin());

        // 校驗：sum [0..25]
        uint8_t sum = 0;
        for (int i=0;i<=25;i++) sum += fr[i];
        uint8_t rx_chk = fr[26];
        if (sum != rx_chk) {
            dump_bytes_throttled(rxbuf, "[RX] checksum mismatch, drop one byte");
            rxbuf.erase(rxbuf.begin()+p);
            continue;
        }

        // 解析 payload
        auto U32 = [&](int idx)->int32_t{
            return (int32_t)( fr[idx] | (fr[idx+1]<<8) | (fr[idx+2]<<16) | (fr[idx+3]<<24) );
        };
        R_H1 = U32(4);
        R_L1 = U32(8);
        R_En1 = fr[20];
        R_En2 = fr[21];
        R_C1  = fr[24];

        last_rx_time = ros::Time::now();
        Flag_OK = 1; ++b; ++a; parsed_any = true;

        // 吃掉這幀
        rxbuf.erase(rxbuf.begin()+p, rxbuf.begin()+p+FRAME_SZ);
    }

    if (++count_1 > 149 && log) {
        count_1 = 0;
        std::cout<< "[01] Current_Height_1:" << (int)R_H1 <<"[mm]\n";
        std::cout<< "[02] Current_length_1:" << (int)R_L1 <<"[mm]\n";
        std::cout<< "[05] (Height_1)En1:"    << (int)R_En1  <<"\n";
        std::cout<< "[06] (length_1)En2:"    << (int)R_En2  <<"\n";
        std::cout<< "[09] State_Claw1:"      << (int)R_C1   <<"\n";
        std::cout<< "[12] 主循環頻數a:"      << (uint16_t)a <<"\n";
        std::cout<< "[13] 有效接收數b:"      << (uint16_t)b <<"\n";
        std::cout<< "[14] a/b:"              << (b? (float)a/b : 0.0f) <<"\n";
        if (b>5000) { b/=10; a/=10; }
        std::cout<< "-----------------------\n";
    }

    // 若有資料堆著卻都湊不到幀，印一次 hexdump 幫定位協議問題
    if (!parsed_any && n>0 && rxbuf.size()>=8) {
        dump_bytes_throttled(rxbuf, "[RX dbg] bytes in buffer");
    }
}

// ============== 初始化下發一次 ==============
void initialize_arms()
{
    S_H1 = 0;
    S_L1 = 10;
    S_En1 = 1;
    S_En2 = 1;
    S_C1 = 0;
    last_rx_time = ros::Time(0); // 尚未收到
    send_data();
}

//--- MinimalSubscriber (保留原樣) ---//
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

//--- ArmStatusPublisher：無 RX 時 mirror 最後下發值 ---//
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

        const double RX_TIMEOUT = 0.5;
        bool stale = last_rx_time.isZero() || ((ros::Time::now() - last_rx_time).toSec() > RX_TIMEOUT);

        if (stale) { // 沒有新回傳，用最後下發值回報給 UI
            msg.height1 = S_H1;
            msg.length1 = S_L1;
            msg.claw1   = (S_C1 != 0);
        } else {
            msg.height1 = R_H1;
            msg.length1 = R_L1;
            msg.claw1   = (R_C1 != 0);
        }

        msg.speed_h1 = speed_h1;
        msg.speed_l1 = speed_l1;
        msg.speed_h2 = speed_h2;
        msg.speed_l2 = speed_l2;
        pub_.publish(msg);
    }
};

// -------- CmdCutPliersPublisher：處理 UI 指令，定期下發 --------
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
        nh.param<bool>("allow_retract", allow_retract_, true); // 預設允許後退，配合 UI

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
    int target_length_;      // 目標長度（-1 表示尚未設定）
    int last_valid_length_;  // MCU 回報/或 UI 追蹤到的最後有效長度
    bool claw_;
    bool allow_retract_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;

    void cmdCutPliersCallback(const cut_pliers_controller::CmdCutPliers::ConstPtr& msg)
    {
        bool updated = false;

        // 長度（mode=0 前進 / mode=1 後退）
        if (msg->length1 >= 0) {
            if (msg->mode == 0) { // forward
                if (!allow_retract_) {
                    // 僅允許 >= 當前
                    if (msg->length1 >= last_valid_length_) {
                        target_length_ = msg->length1;
                        last_valid_length_ = target_length_;
                        updated = true;
                    } else {
                        ROS_WARN("Forward-only: ignore len=%d < current=%d",
                                 msg->length1, last_valid_length_);
                    }
                } else {
                    target_length_ = msg->length1;
                    last_valid_length_ = std::max(last_valid_length_, target_length_);
                    updated = true;
                }
            } else { // reverse
                if (allow_retract_) {
                    target_length_ = msg->length1;
                    last_valid_length_ = msg->length1;
                    updated = true;
                } else {
                    ROS_WARN("Retract disabled: ignore reverse to %d", msg->length1);
                }
            }
        }

        // 高度
        if (msg->height1 >= 0 && target_height_ != msg->height1) {
            target_height_ = msg->height1;
            updated = true;
        }

        // 爪子
        if (msg->claw1 != claw_) {
            claw_ = msg->claw1;
            updated = true;
        }

        if (updated) {
            ROS_INFO("Updated target: height=%d, length=%d, mode=%d",
                     target_height_, target_length_, msg->mode);
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        // 高度：夾在 [0,280]
        S_H1 = std::min(std::max(target_height_, 0), 280);

        // 長度：若尚未設定 target_length_，用 last_valid_length_
        int length_to_send = (target_length_ >= 0 ? target_length_ : last_valid_length_);
        S_L1 = std::min(std::max(length_to_send, 10), 440);

        // 爪子
        S_C1 = claw_ ? 1 : 0;

        send_data();
    }
};

// -----------------------------------------------------------------------------
// 主函式
// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cut_pliers_controller_arm1");
    ros::NodeHandle nh("~");

    // 讀取串口參數
    std::string port;
    int baudrate, timeout;
    bool log;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<int>("timeout", timeout, 100);
    nh.param<bool>("log", log, false);

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
        receive_and_process_data(log);
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
