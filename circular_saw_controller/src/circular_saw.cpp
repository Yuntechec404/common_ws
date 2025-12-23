#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdint>

#include "circular_saw_controller/CircularSawCmd.h"
#include <circular_saw_controller/CircularSawState.h>

// ============================================================
//  座標定義
//  ROS 端： X 越大 = 越往前； Z 越大 = 越往下
//  MCU 端：原始韌體 MOVE() 期待：X(length) 0~-400, Z(height) 0~-350，0在原點
//  => ROS->MCU 送下去要取負號；MCU->ROS 回來再取負號
// ============================================================

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return std::max(lo, std::min(hi, v));
}

// ROS(+X forward, +Z down) -> MCU(0..-400, 0..-350)
static inline int32_t ros2mcu_x(int32_t x_ros_mm) { return -clamp_i32(x_ros_mm, 0, 400); }
static inline int32_t ros2mcu_z(int32_t z_ros_mm) { return -clamp_i32(z_ros_mm, 0, 350); }

// MCU -> ROS
static inline int32_t mcu2ros_x(int32_t x_mcu_mm) { return x_mcu_mm; }
static inline int32_t mcu2ros_z(int32_t z_mcu_mm) { return z_mcu_mm; }

// ============================================================
// serial
// ============================================================
static serial::Serial sp;

// TX flag
static volatile uint8_t FLAG_USART = 0;

// ============================================================
// Command/Target variables to MCU
// ============================================================
static int32_t S_H1 = 0; // MCU height (0..-350)
static int32_t S_L1 = 0; // MCU length (0..-400)

static uint8_t S_En1 = 1;
static uint8_t S_En2 = 1;
static uint8_t S_En3 = 1;

static float  Servo_Angle_SE = 0.0f;
static int16_t Saw_speed_SE  = 0;
static int16_t Move_speed_L  = 3000;
static int16_t Move_speed_H  = 2000;

// ============================================================
// Feedback from MCU
// ============================================================
static int32_t R_H1 = 0;
static int32_t R_L1 = 0;

static uint8_t R_En1 = 0;
static uint8_t R_En2 = 0;
static uint8_t R_En3 = 0;

static float Servo_Angle_RE = 0.0f;
static int32_t Voltage = 0;

// counters
static uint16_t a = 0, b = 0;
static uint16_t log_div = 0;

// ============================================================
// params
// ============================================================
static bool enable_log = false;

static std::string port_name = "/dev/ttyUSB0";
static int baudrate = 115200;
static int timeout_ms = 100;
static std::string cmd_topic = "/circular_saw_cmd";
static std::string state_topic = "/circular_saw_state";

// boot default target (ROS semantics: +X forward, +Z down)
static int32_t target_z_param = 0;
static int32_t target_x_param = 0;
static float target_angle_param = 0.0f;

static bool stop_latched = false;
// ============================================================
// send frame to MCU
// ============================================================
static void send_data()
{
    uint8_t tbuf[26] = {0};

    // Header / Protocol
    tbuf[0] = 0xAA;
    tbuf[1] = 0xAA;
    tbuf[2] = 0xF1;
    tbuf[3] = 21; // payload length

    // H1 (int32)
    tbuf[4]  = (uint8_t)(S_H1 >> 0);
    tbuf[5]  = (uint8_t)(S_H1 >> 8);
    tbuf[6]  = (uint8_t)(S_H1 >> 16);
    tbuf[7]  = (uint8_t)(S_H1 >> 24);

    // L1 (int32)
    tbuf[8]  = (uint8_t)(S_L1 >> 0);
    tbuf[9]  = (uint8_t)(S_L1 >> 8);
    tbuf[10] = (uint8_t)(S_L1 >> 16);
    tbuf[11] = (uint8_t)(S_L1 >> 24);

    // Servo angle *100 (int32)
    int32_t ang100 = (int32_t)(Servo_Angle_SE * 100.0f);
    tbuf[12] = (uint8_t)(ang100 >> 0);
    tbuf[13] = (uint8_t)(ang100 >> 8);
    tbuf[14] = (uint8_t)(ang100 >> 16);
    tbuf[15] = (uint8_t)(ang100 >> 24);

    // Saw speed (int16)
    tbuf[16] = (uint8_t)(Saw_speed_SE >> 0);
    tbuf[17] = (uint8_t)(Saw_speed_SE >> 8);

    // Move speeds (int16)
    tbuf[18] = (uint8_t)(Move_speed_H >> 0);
    tbuf[19] = (uint8_t)(Move_speed_H >> 8);
    tbuf[20] = (uint8_t)(Move_speed_L >> 0);
    tbuf[21] = (uint8_t)(Move_speed_L >> 8);

    // Enables
    tbuf[22] = S_En1;
    tbuf[23] = S_En2;
    tbuf[24] = S_En3;

    // checksum: sum of [0..24] into [25]
    uint8_t sum = 0;
    for (int i = 0; i < 25; i++) sum += tbuf[i];
    tbuf[25] = sum;

    try
    {
        sp.write(tbuf, 26);
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to send data through serial port");
    }

    FLAG_USART = 0; // important: clear after sending
}

// ============================================================
// ROS callback
// ============================================================
static void cmdCallback(const circular_saw_controller::CircularSawCmd::ConstPtr& msg)
{
  if (msg->stop) {
    // STOP：速度=0、目標=當下位置（只做一次）、不關 EN
    Saw_speed_SE = 0;
    Move_speed_H = 0;
    Move_speed_L = 0;

    // 不關 EN（維持力矩）
    if (S_En1 == 0 || S_En2 == 0 || S_En3 == 0) {
      S_En1 = 1; S_En2 = 1; S_En3 = 1;
    }

    // 只做一次：把目標更新為「目前回授位置」並發送
    if (!stop_latched) {
      // R_H1/R_L1 是 MCU 回傳的 ROS 語意位置(mm, 正值)
      // 你要：STOP 當下把目標鎖住在現況
      S_H1 = clamp_i32(ros2mcu_z(R_H1), -350, 0);
      S_L1 = clamp_i32(ros2mcu_x(R_L1), -400, 0);

      stop_latched = true;
      FLAG_USART = 1;     // ✅ 送出一次（速度=0 + 目標=當下位置）
    }

    return;
  }

  // RUN：解除 stop latch（讓下一次 STOP 能再做一次 latch）
  stop_latched = false;

  // === RUN 不變 ===
  int32_t x_ros = clamp_i32(msg->x_pos, 0, 400);
  int32_t z_ros = clamp_i32(msg->z_pos, 0, 350);

  S_L1 = clamp_i32(ros2mcu_x(x_ros), -400, 0);
  S_H1 = clamp_i32(ros2mcu_z(z_ros), -350, 0);

  Servo_Angle_SE = msg->angle;
  Move_speed_L = (int16_t)clamp_i32(msg->x_speed, 0, 5000);
  Move_speed_H = (int16_t)clamp_i32(msg->z_speed, 0, 5000);
  Saw_speed_SE = (int16_t)clamp_i32(msg->saw_speed, 0, 6000);

  if (S_En1 == 0 || S_En2 == 0 || S_En3 == 0) {
    S_En1 = 1; S_En2 = 1; S_En3 = 1;
  }

  FLAG_USART = 1;
}

// ============================================================
// robust frame parser (MCU -> PC): 24 bytes
//  [0]=AA [1]=AA [2]=F1 [3]=19 ... [23]=checksum
//  checksum = sum([0..22])
// ============================================================
static bool try_parse_one_frame(std::vector<uint8_t>& rx, std::array<uint8_t,24>& out)
{
    // find sync 0xAA 0xAA
    while (rx.size() >= 2)
    {
        if (rx[0] == 0xAA && rx[1] == 0xAA) break;
        rx.erase(rx.begin());
    }
    if (rx.size() < 24) return false;

    // candidate
    for (int i=0;i<24;i++) out[i]=rx[i];

    // basic check
    if (!(out[0]==0xAA && out[1]==0xAA && out[2]==0xF1 && out[3]==19))
    {
        // bad sync, drop 1 byte
        rx.erase(rx.begin());
        return false;
    }

    // checksum check
    uint8_t sum = 0;
    for (int i=0;i<23;i++) sum += out[i];
    if (out[23] != sum)
    {
        // bad checksum, drop 1 byte and retry next loop
        rx.erase(rx.begin());
        return false;
    }

    // consume 24 bytes
    rx.erase(rx.begin(), rx.begin()+24);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_saw");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // params
    pnh.param<std::string>("port", port_name, std::string("/dev/ttyUSB1"));
    pnh.param<int>("baudrate", baudrate, 115200);
    pnh.param<int>("timeout", timeout_ms, 100);
    pnh.param<std::string>("cmd_topic", cmd_topic, std::string("/circular_saw_cmd"));
    pnh.param<std::string>("state_topic", state_topic, std::string("/circular_saw_state"));
    pnh.param<int>("target_height", target_z_param, 0);   // ROS語意：Z(往下) mm
    pnh.param<int>("target_length", target_x_param, 0);   // ROS語意：X(往前) mm
    pnh.param<float>("target_angle", target_angle_param, 0.0f);
    pnh.param<bool>("log", enable_log, false);

    ROS_INFO_STREAM("circular_saw config:");
    ROS_INFO_STREAM("port = " << port_name);
    ROS_INFO_STREAM("baudrate = " << baudrate);
    ROS_INFO_STREAM("timeout_ms = " << timeout_ms);
    ROS_INFO_STREAM("cmd_topic = " << cmd_topic);
    ROS_INFO_STREAM("state_topic = " << state_topic);
    ROS_INFO_STREAM("target_x(ROS) = " << target_x_param << " (X bigger -> forward)");
    ROS_INFO_STREAM("target_z(ROS) = " << target_z_param << " (Z bigger -> down)");
    ROS_INFO_STREAM("target_angle = " << target_angle_param);
    ROS_INFO_STREAM("log = " << (enable_log ? "true" : "false"));

    ros::Subscriber sub = nh.subscribe(cmd_topic, 10, cmdCallback);
    ros::Publisher pub = nh.advertise<circular_saw_controller::CircularSawState>(state_topic, 10);

    // serial open
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms);
    sp.setPort(port_name);
    sp.setBaudrate(baudrate);
    sp.setTimeout(to);

    try { sp.open(); }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port: " << port_name);
        return -1;
    }

    if (!sp.isOpen())
        return -1;

    ROS_INFO_STREAM(port_name << " is opened.");

    // init default target
    // ROS->MCU mapping
    S_L1 = ros2mcu_x(target_x_param);
    S_H1 = ros2mcu_z(target_z_param);
    Servo_Angle_SE = target_angle_param;

    Saw_speed_SE  = 0;
    Move_speed_H  = 2000;
    Move_speed_L  = 3000;
    S_En1 = S_En2 = S_En3 = 1;

    ros::Rate loop_rate(100);

    // RX buffer
    std::vector<uint8_t> rxbuf;
    rxbuf.reserve(1024);

    while (ros::ok())
    {
        ros::spinOnce();

        if (FLAG_USART == 1)
            send_data();

        // read all available bytes
        size_t n = sp.available();
        a++;

        if (n > 0)
        {
            std::vector<uint8_t> tmp(n);
            size_t got = sp.read(tmp.data(), n);
            tmp.resize(got);
            rxbuf.insert(rxbuf.end(), tmp.begin(), tmp.end());
        }

        // parse frames as many as possible
        std::array<uint8_t,24> frame{};
        while (try_parse_one_frame(rxbuf, frame))
        {
            b++;

            // parse payload
            int32_t H1 = (int32_t)((frame[4]  << 0)  | (frame[5]  << 8)  | (frame[6]  << 16) | (frame[7]  << 24));
            int32_t L1 = (int32_t)((frame[8]  << 0)  | (frame[9]  << 8)  | (frame[10] << 16) | (frame[11] << 24));

            uint8_t En1 = frame[12];
            uint8_t En2 = frame[13];
            uint8_t En3 = frame[14];

            float angle = (float)((int32_t)((frame[15] << 0) | (frame[16] << 8) | (frame[17] << 16) | (frame[18] << 24))) * 0.01f;
            int32_t vol = (int32_t)((frame[19] << 0) | (frame[20] << 8) | (frame[21] << 16) | (frame[22] << 24));

            // ---- 異常值濾除（很重要，避免亂跳）----
            // MCU 規格：H in [ 0, 350 ], L in [ 0, 400 ]  (0在原點)
            if (H1 < 0 || H1 > 350) continue;
            if (L1 < 0 || L1 > 400) continue;
            if (angle > 120.0f || angle < -120.0f) continue;    // 放寬一點，避免偶發抖動
            if (vol < 0 || vol > 5000) continue;                // 0~50.00V (依你除以100)

            // accept
            R_H1 = H1;
            R_L1 = L1;
            R_En1 = En1;
            R_En2 = En2;
            R_En3 = En3;
            Servo_Angle_RE = angle;
            Voltage = vol;

            // publish state (ROS semantics)
            circular_saw_controller::CircularSawState state_msg;
            state_msg.x_pos = mcu2ros_x(R_L1);   // 你要：X越大越往前
            state_msg.z_pos = mcu2ros_z(R_H1);   // 你要：Z越大越往下
            state_msg.angle = Servo_Angle_RE;

            // MCU 沒回傳 saw speed，就維持「最後下發值」作為顯示
            state_msg.saw_speed = Saw_speed_SE;

            state_msg.en1 = (R_En1 != 0);
            state_msg.en2 = (R_En2 != 0);
            state_msg.en3 = (R_En3 != 0);
            state_msg.voltage = (float)Voltage / 100.0f;
            state_msg.frame_count = b;

            pub.publish(state_msg);
        }

        // log
        if (enable_log)
        {
            log_div++;
            if (log_div >= 10) // ~10Hz
            {
                log_div = 0;
                ROS_INFO_STREAM("[MCU] H=" << R_H1 << "mm, L=" << R_L1 << "mm, angle=" << Servo_Angle_RE
                                << ", V=" << (float)Voltage/100.0f
                                << ", EN=" << (int)R_En1 << "," << (int)R_En2 << "," << (int)R_En3
                                << ", frames=" << b);
                ROS_INFO_STREAM("[ROS] x=" << mcu2ros_x(R_L1) << "mm (forward+), z=" << mcu2ros_z(R_H1) << "mm (down+)");
            }
        }

        loop_rate.sleep();
    }

    sp.close();
    return 0;
}
