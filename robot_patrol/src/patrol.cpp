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

void Patrol::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_data) {

  std::vector<float> ranges = laser_data->ranges;
  int num_ranges = ranges.size();

  // Middle index of the laser scan data
  int middle_index = num_ranges / 2;
  // Number of indices corresponding to 90 degrees on either side
  int indices_for_90_degrees =
      static_cast<int>(M_PI / 2 / laser_data->angle_increment);

  // Calculate start and end index for the 180 degree view
  int start_index = std::max(middle_index - indices_for_90_degrees,
                             0); // Ensure we don't go below zero
  int end_index =
      std::min(middle_index + indices_for_90_degrees,
               num_ranges - 1); // Ensure we don't go past the array size

  double obstacle_distance_threshold = 0.35; // 35 cm
  int window_size = 100;

  // Flag to determine if an obstacle is detected within the desired range
  bool obstacle_detected = false;

  for (int i = start_index; i <= end_index; i++) {
    if (!std::isinf(ranges[i]) && ranges[i] < obstacle_distance_threshold) {
      obstacle_detected = true;
      break; // Stop checking once an obstacle is found
    }
  }
  // Check for obstacle directly in front
  if (obstacle_detected) {
    // Obstacle detected, find safest direction
    double max_distance = 0.0;
    int max_index = 0;

    // Only consider the front 180 degrees
    for (int i = start_index; i <= end_index; i++) {
      int window_start = std::max(i - window_size / 2, start_index);
      int window_end = std::min(i + window_size / 2, end_index);
      double window_min_distance = std::numeric_limits<double>::max();

      for (int j = window_start; j <= window_end; j++) {
        if (!std::isinf(ranges[j]) && ranges[j] < window_min_distance) {
          window_min_distance = ranges[j];
        }
      }

      if (window_min_distance > max_distance) {
        max_distance = window_min_distance;
        max_index = i;
      }
    }

    // Calculate the safest angle
    double safest_angle =
        laser_data->angle_min + max_index * laser_data->angle_increment;
    if (safest_angle < -M_PI / 2) {
      safest_angle = -M_PI / 2;
    } else if (safest_angle > M_PI / 2) {
      safest_angle = M_PI / 2;
    }

    // Store this angle in the class variable
    direction_.store(safest_angle);
  } else {
    direction_.store(0.0);
  }
}

void Patrol::publishCommand() {
  //RCLCPP_INFO(this->get_logger(), "Publishing command...");

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.1; // Always move forward at 0.1 m/s

  cmd.angular.z = (direction_ / 1.4);
  //RCLCPP_INFO(this->get_logger(), "direction_/2 = %f", direction_ / 2.0);

  publisher_->publish(cmd);

  // It sleeps for wait_time_ seconds to maintain the control loop at 10 Hz.
  // sleep(this->wait_time_);

  //RCLCPP_INFO(this->get_logger(), "Command published successfully.");
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
