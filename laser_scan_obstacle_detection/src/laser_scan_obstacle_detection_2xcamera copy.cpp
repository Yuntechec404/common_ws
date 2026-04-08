// src/laser_scan_obstacle_detection_2xcamera.cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <limits>
#include <string>
#include <cmath>

static ros::Publisher cmd_out_pub;
static geometry_msgs::Twist last_cmd_in;

static double stop_distance = 0.8;
static double pause_seconds = 10.0;
static double cooldown_seconds = 10.0;
static double pub_rate_hz   = 20.0;

static float front_min_distance = std::numeric_limits<float>::infinity();
static float back_min_distance  = std::numeric_limits<float>::infinity();

static bool got_front = false, got_back = false, got_cmd = false;
static ros::Time last_front_t, last_back_t, last_cmd_t;
static double scan_timeout = 0.5;
static double cmd_timeout  = 0.5;

static ros::Time pause_until(0);               // 暫停到什麼時候（now < pause_until => 送0）
static ros::Time next_allowed_trigger_time(0); // 允許下一次觸發暫停的時間點（冷卻截止）

static geometry_msgs::Twist zeroTwist()
{
  geometry_msgs::Twist t;
  t.linear.x = t.linear.y = t.linear.z = 0.0;
  t.angular.x = t.angular.y = t.angular.z = 0.0;
  return t;
}

static float minValidRange(const sensor_msgs::LaserScan& scan)
{
  float m = std::numeric_limits<float>::infinity();
  for (float r : scan.ranges)
  {
    if (!std::isfinite(r)) continue;
    if (r < scan.range_min || r > scan.range_max) continue;
    if (r < m) m = r;
  }
  return m;
}

static void FrontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  front_min_distance = minValidRange(*scan);
  got_front = true;
  last_front_t = scan->header.stamp.isZero() ? ros::Time::now() : scan->header.stamp;
}

static void BackScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  back_min_distance = minValidRange(*scan);
  got_back = true;
  last_back_t = scan->header.stamp.isZero() ? ros::Time::now() : scan->header.stamp;
}

static void CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  last_cmd_in = *cmd;
  got_cmd = true;
  last_cmd_t = ros::Time::now();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_scan_obstacle_detection_2xcamera");
  ros::NodeHandle nh, pnh("~");

  std::string front_scan_topic = "/front_scan_filtered";
  std::string back_scan_topic  = "/back_scan_filtered";
  std::string cmd_in_topic     = "/cmd_vel_nav";
  std::string cmd_out_topic    = "/cmd_vel";

  pnh.param("stop_distance", stop_distance, 0.8);
  pnh.param("pause_seconds", pause_seconds, 10.0);
  pnh.param("cooldown_seconds", cooldown_seconds, 10.0);
  pnh.param("pub_rate_hz",   pub_rate_hz,   20.0);
  pnh.param("scan_timeout",  scan_timeout,  0.5);
  pnh.param("cmd_timeout",   cmd_timeout,   0.5);

  pnh.param("front_scan_topic", front_scan_topic, front_scan_topic);
  pnh.param("back_scan_topic",  back_scan_topic,  back_scan_topic);
  pnh.param("cmd_in_topic",      cmd_in_topic,     cmd_in_topic);
  pnh.param("cmd_out_topic",     cmd_out_topic,    cmd_out_topic);

  ros::Subscriber sub_front = nh.subscribe<sensor_msgs::LaserScan>(front_scan_topic, 10, FrontScanCallback);
  ros::Subscriber sub_back  = nh.subscribe<sensor_msgs::LaserScan>(back_scan_topic,  10, BackScanCallback);
  ros::Subscriber sub_cmd   = nh.subscribe<geometry_msgs::Twist>(cmd_in_topic,       10, CmdCallback);

  cmd_out_pub = nh.advertise<geometry_msgs::Twist>(cmd_out_topic, 10);

  ros::Rate rate(pub_rate_hz);

  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();

    // 1) 資料有效性檢查（你原本有的 fail-safe，保留）
    const bool front_ok = got_front && (now - last_front_t).toSec() <= scan_timeout;
    const bool back_ok  = got_back  && (now - last_back_t ).toSec() <= scan_timeout;
    const bool cmd_ok   = got_cmd   && (now - last_cmd_t  ).toSec() <= cmd_timeout;

    if (!front_ok || !back_ok || !cmd_ok)
    {
      cmd_out_pub.publish(zeroTwist());
      rate.sleep();
      continue;
    }

    // 2) 障礙判斷
    const bool obstacle_now =
      (front_min_distance <= stop_distance) || (back_min_distance <= stop_distance);

    // 3) 暫停期間：一直送 0（最高優先權）
    if (now < pause_until)
    {
      cmd_out_pub.publish(zeroTwist());
      rate.sleep();
      continue;
    }

    // 4) 非暫停期間：若偵測到障礙，且「已過冷卻期」才允許觸發新一輪暫停
    if (obstacle_now && now >= next_allowed_trigger_time)
    {
      pause_until = now + ros::Duration(pause_seconds);

      // 關鍵：下一次允許觸發 = 暫停結束 + 冷卻期
      next_allowed_trigger_time = pause_until + ros::Duration(cooldown_seconds);

      ROS_WARN("Obstacle detected. Pause %.1fs, then cooldown %.1fs (forwarding cmd_vel during cooldown).",
               pause_seconds, cooldown_seconds);

      cmd_out_pub.publish(zeroTwist());
      rate.sleep();
      continue;
    }

    // 5) 其餘情況：正常轉發 /cmd_vel_nav -> /cmd_vel
    cmd_out_pub.publish(last_cmd_in);
    rate.sleep();
  }

  return 0;
}
