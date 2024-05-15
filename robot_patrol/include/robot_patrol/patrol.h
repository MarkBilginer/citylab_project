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

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void publishCommand();
  std::pair<int, int> calculateIndexRange(int range_size);
  std::pair<float, int>
  findMinimumDistance(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                      int start_index, int end_index);
  void
  identifySafestDirection(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                          int start_index, int end_index, int range_size);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<float> direction_;
  float min_distance_;
  const std::chrono::milliseconds wait_time_{100}; // 10Hz
};

#endif // PATROL_H
