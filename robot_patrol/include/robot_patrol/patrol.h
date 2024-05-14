// patrol.h
#ifndef PATROL_H
#define PATROL_H

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <atomic>
#include <cmath>
#include <limits>
#include <unistd.h>

class Patrol : public rclcpp::Node {
public:
  Patrol();
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
  void publishCommand();
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<float> direction_;
  float wait_time_ = 0.1;
};

#endif // PATROL_H
