#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using GetDirection = custom_interfaces::srv::GetDirection;

class DirectionClient : public rclcpp::Node {
public:
  DirectionClient() : Node("test_service_client_node") {
    // Initialize the subscriber to laser scan data
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&DirectionClient::laser_callback, this,
                  std::placeholders::_1));

    // Initialize the service client for getting direction
    client_ = this->create_client<GetDirection>("/direction_service");
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service /direction_service not available after waiting");
      return;
    }

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;

    auto result = client_->async_send_request(
        request, std::bind(&DirectionClient::response_callback, this,
                           std::placeholders::_1));
  }

  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Suggested direction: %s",
                response->direction.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<GetDirection>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
