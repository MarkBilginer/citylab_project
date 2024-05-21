#ifndef PATROL_WITH_SERVICE_H
#define PATROL_WITH_SERVICE_H

#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>
#include <mutex>
#include <unistd.h>

class Patrol : public rclcpp::Node {
public:
  Patrol();
  bool initialize();
  void startTimer();

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void publishCommand(const std::string &direction);
  void handleServiceFailure();
  void handleNoDirection();
  void switchToSafeMode();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  float min_distance_;
  const std::chrono::milliseconds wait_time_{100}; // 10Hz
  sensor_msgs::msg::LaserScan last_laser_;
  std::mutex mutex_;
  static constexpr int MAX_RETRIES_ = 3;
  int retry_count_ = 0;
};

#endif // PATROL_WITH_SERVICE_H
