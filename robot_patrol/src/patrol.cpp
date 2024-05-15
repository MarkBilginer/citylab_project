#include "robot_patrol/patrol.h"
#include <unistd.h>

Patrol::Patrol() : Node("robot_patrol_node"), direction_(0.0) {

  timer_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  scan_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions scan_options;
  scan_options.callback_group = scan_callback_group_;
  subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Patrol::laserCallback, this, std::placeholders::_1),
      scan_options);

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ = this->create_wall_timer(this->wait_time_,
                                   std::bind(&Patrol::publishCommand, this),
                                   timer_callback_group_); // timer for 10hz
}

double radiansToDegrees(double radians) { return radians * (180.0 / M_PI); }

double degreesToRadians(double degrees) { return degrees * (M_PI / 180.0); }

void Patrol::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_data) {
  std::vector<float> ranges = laser_data->ranges;

  int window_size = 100; // Size of the window to check around each angle
  double safest_angle = 0.0;
  double max_minimum_distance = -1.0;

  // The actual indices should be adapted based on your laser scanner specs
  int middle_index = ranges.size() / 2;
  int half_window = window_size / 2;

  for (int i = middle_index - (window_size * 2);
       i <= middle_index + (window_size * 2); ++i) {
    int start_index = i - half_window;
    int end_index = i + half_window;

    if (start_index < 0 || static_cast<size_t>(end_index) >= ranges.size()) {
      continue; // Skip if the window extends beyond the range of indices
    }

    double min_distance = std::numeric_limits<double>::max();

    for (int j = start_index; j <= end_index; ++j) {
      double range = ranges[j];
      if (!std::isinf(range) && range < min_distance) {
        min_distance = range;
      }
    }

    if (min_distance > max_minimum_distance) {
      max_minimum_distance = min_distance;
      safest_angle = laser_data->angle_min + i * laser_data->angle_increment;
    }
  }

  direction_.store(
      safest_angle); // Store the safest angle in an atomic variable
  RCLCPP_DEBUG(
      this->get_logger(),
      "Safe direction set at angle [%f] with max minimum distance [%f]",
      radiansToDegrees(safest_angle), max_minimum_distance);
}

void Patrol::publishCommand() {
  RCLCPP_INFO(this->get_logger(), "Publishing command...");

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.1; // Always move forward at 0.1 m/s

  // Base proportional gain
  // double base_kp = 0.1;

  // Boosting factor that decreases as distance increases
  // double min_distance_boost =
  // std::exp(5.0 / std::max(0.1, static_cast<double>(min_distance_)));

  // Adaptive proportional gain
  // double adaptive_kp = base_kp * min_distance_boost;

  // Max angular velocity
  // double max_angular_velocity = 1.5;

  // cmd.angular.z = std::clamp(adaptive_kp * direction_, -max_angular_velocity,
  //                          max_angular_velocity);

  // cmd.angular.z = direction_ / 2;

  // RCLCPP_INFO(this->get_logger(),
  //"Command values - Linear X: %f, Angular Z: %f", cmd.linear.x,
  // cmd.angular.z);
  // velocity_msg.angular.z = (direction_/1.5); // simulation (COMMENT WHEN
  // USING ACTUAL ROBOT)
  cmd.angular.z =
      (direction_ / 2.0);
  RCLCPP_DEBUG(this->get_logger(), "direction_/2 = %f", direction_ / 2);

  publisher_->publish(cmd);

  // It sleeps for wait_time_ seconds to maintain the control loop at 10 Hz.
  // sleep(this->wait_time_);

  RCLCPP_INFO(this->get_logger(), "Command published successfully.");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto robot_patrol_node = std::make_shared<Patrol>();
  RCLCPP_INFO(robot_patrol_node->get_logger(), "Patrol INFO...");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_patrol_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
