// patrol.cpp
#include "robot_patrol/patrol.h"
#include <unistd.h>
using namespace std::chrono_literals;

Patrol::Patrol() : Node("robot_patrol_node"), direction_(0.0) {

  auto subscriber_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // auto timer_callback_group =
  //   this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = subscriber_callback_group;
  subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Patrol::laserCallback, this, std::placeholders::_1),
      subscription_options);

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ =
      this->create_wall_timer(500ms, std::bind(&Patrol::publishCommand, this));
}

double radiansToDegrees(double radians) { return radians * (180.0 / M_PI); }

double degreesToRadians(double degrees) { return degrees * (M_PI / 180.0); }

void Patrol::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

  RCLCPP_INFO(this->get_logger(), "Laser callback triggered.");

  static auto last_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();

  // Check if at least 0.1 second has passed since the last processing = 10 hz
  if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_time)
          .count() > this->wait_time_) {
    int range_size = scan->ranges.size();
    int start_index = range_size / 4;   // Assuming this is -45 degrees
    int end_index = 3 * range_size / 4; // Assuming this is +45 degrees

    RCLCPP_INFO(
        this->get_logger(),
        "Processing scan with range size: %d, Start index: %d, End index: %d",
        range_size, start_index, end_index);

    float min_distance = std::numeric_limits<float>::infinity();
    int min_distance_index = -1; // Initialize to an invalid index

    for (int i = start_index; i <= end_index; i++) {
      if (scan->ranges[i] < min_distance && !std::isinf(scan->ranges[i])) {
        min_distance = scan->ranges[i];
        min_distance_index = i;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Minimum distance detected: %f at index %d",
                min_distance, min_distance_index);

    if (min_distance < 0.35) {
      int safest_index = -1;        // Initialize to an invalid index
      float max_safe_distance = -1; // Start with a negative distance to ensure
                                    // any valid range is larger

      for (int i = start_index; i <= end_index; i++) {
        // Check both that the range is greater than any previously found and
        // that it's not infinite
        if (scan->ranges[i] > max_safe_distance &&
            !std::isinf(scan->ranges[i])) {
          max_safe_distance = scan->ranges[i];
          safest_index = i;
        }
      }

      if (safest_index != -1) { // Ensure a valid index was found
        float angle = (safest_index - range_size / 2) * scan->angle_increment;
        direction_ = angle;
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle detected close by. Safest index: %d, Safest "
                    "angle: %f, Distance: %f, Angle increment: %f",
                    safest_index, angle, max_safe_distance,
                    scan->angle_increment);
      } else {
        direction_ = 0; // Fallback if no valid direction found
        RCLCPP_INFO(this->get_logger(), "No safe direction found due to all "
                                        "infinite distances or other issues.");
      }
    } else {
      direction_ = 0;
      RCLCPP_INFO(this->get_logger(),
                  "No immediate obstacle detected. Moving forward.");
    }
    last_time = current_time; // Update last processing time
  }
}

void Patrol::publishCommand() {
  RCLCPP_INFO(this->get_logger(), "Publishing command...");

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.1; // Always move forward at 0.1 m/s

  double Kp = 0.2;                   // Proportional gain
  double max_angular_velocity = 1.5; // Max angular velocity
  cmd.angular.z =
      std::clamp(Kp * direction_, -max_angular_velocity, max_angular_velocity);

  RCLCPP_INFO(this->get_logger(),
              "Command values - Linear X: %f, Angular Z: %f", cmd.linear.x,
              cmd.angular.z);

  publisher_->publish(cmd);
  // It sleeps for wait_time_ seconds to maintain the control loop at 10 Hz.
  sleep(this->wait_time_);

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
