#include "robot_patrol/patrol_with_service.h"

using namespace std::chrono_literals;

Patrol::Patrol() : Node("robot_patrol_with_service_client_node") {

  timer_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  scan_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  client_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions scan_options;
  scan_options.callback_group = scan_callback_group_;
  subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Patrol::laserCallback, this, std::placeholders::_1),
      scan_options);

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  client_ = this->create_client<custom_interfaces::srv::GetDirection>(
      "/direction_service", rmw_qos_profile_default, client_callback_group_);

  // timer_ = this->create_wall_timer(this->wait_time_,
  //                                std::bind(&Patrol::publishCommand, this),
  //                              timer_callback_group_); // timer for 10hz
}

void Patrol::startTimer() {
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // Adjust the interval as needed
      std::bind(&Patrol::publishCommand, this), timer_callback_group_);
}

bool Patrol::initialize() {
  auto timeout = std::chrono::seconds(30);
  auto check_interval = std::chrono::seconds(1);
  auto start_time = this->now();

  while (!client_->wait_for_service(check_interval)) {
    if (this->now() - start_time > timeout) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to connect to the service after 30 seconds");
      return false;
    }
    RCLCPP_INFO(this->get_logger(),
                "Waiting for the service to become available...");
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Backoff strategy
  }
  RCLCPP_INFO(this->get_logger(), "Service is now available.");
  return true;
}

void Patrol::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_laser_ = *laser_data;
}

void Patrol::publishCommand() {
  auto request =
      std::make_shared<custom_interfaces::srv::GetDirection::Request>();
  // Make a local copy of last_laser_
  sensor_msgs::msg::LaserScan local_laser;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    request->laser_data = last_laser_;
  }

  // Asynchronously send the request and specify a callback function
  using ServiceResponseFuture =
      rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto response = future.get();
    geometry_msgs::msg::Twist cmd;
    if (response->direction == "forward") {
      cmd.linear.x = 0.1;  // move forward at 0.1 m/s
      cmd.angular.z = 0.0; // no rotation
    } else if (response->direction == "left") {
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.5; // rotate left
    } else if (response->direction == "right") {
      cmd.linear.x = 0.1;
      cmd.angular.z = -0.5; // rotate right
    }
    publisher_->publish(cmd);
  };

  client_->async_send_request(request, response_received_callback);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto robot_patrol_node = std::make_shared<Patrol>();
  RCLCPP_INFO(robot_patrol_node->get_logger(), "Patrol INFO...");

  if (!robot_patrol_node->initialize()) {
    RCLCPP_ERROR(robot_patrol_node->get_logger(),
                 "Initialization failed, shutting down...");
    rclcpp::shutdown();
    return 1;
  }

  robot_patrol_node->startTimer();

  RCLCPP_INFO(robot_patrol_node->get_logger(),
              "Patrol node initialized successfully.");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_patrol_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
