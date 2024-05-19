#include "custom_interfaces/srv/get_direction.hpp" // Adjust with actual service name
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using GetDirection = custom_interfaces::srv::GetDirection;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_server_node") {
    service_ = this->create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::handle_service, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_service(const std::shared_ptr<GetDirection::Request> request,
                      std::shared_ptr<GetDirection::Response> response) {
    sensor_msgs::msg::LaserScan laser_scan = request->laser_data;
    std::vector<float> ranges = laser_scan.ranges;
    size_t num_points = ranges.size();
    size_t middle_index =
        num_points / 2; // the front of the robot, pointing directly forward
    size_t one_section = num_points / 6; // Each section represents 60 degrees

    // Define section indices for the front 180 degrees
    // Ensure all indices are of type 'size_t' and use static_cast to avoid type
    // mismatch warnings

    size_t front_start = std::max(
        static_cast<size_t>(middle_index - (one_section / 2)), size_t(0));

    size_t front_end = std::max(
        static_cast<size_t>(middle_index + (one_section / 2)), size_t(0));

    size_t left_start = front_end;
    size_t left_end = std::max(
        static_cast<size_t>(middle_index + 3 * (one_section / 2)), size_t(0));

    size_t right_start = std::max(
        static_cast<size_t>(middle_index - 3 * (one_section / 2)), size_t(0));

    size_t right_end = front_start;

    // Initialize total distances for left, front, and right sections as doubles
    double total_dist_sec_left = 0.0, total_dist_sec_front = 0.0,
           total_dist_sec_right = 0.0;

    // Sum distances for left section
    for (size_t i = left_start; i <= left_end; ++i) {
      total_dist_sec_left += ranges[i];
    }

    // Sum distances for front section (centered)
    for (size_t i = front_start; i <= front_end; ++i) {
      total_dist_sec_front += ranges[i];
    }

    // Sum distances for right section
    for (size_t i = right_start; i <= right_end; ++i) {
      total_dist_sec_right += ranges[i];
    }

    RCLCPP_INFO(
        this->get_logger(), "Distance Totals - Left: %f, Front: %f, Right: %f",
        total_dist_sec_left, total_dist_sec_front, total_dist_sec_right);

    // Determine the safest direction based on the largest distance sum
    if (total_dist_sec_left > total_dist_sec_front &&
        total_dist_sec_left > total_dist_sec_right) {
      response->direction = "left";
    } else if (total_dist_sec_right > total_dist_sec_front &&
               total_dist_sec_right > total_dist_sec_left) {
      response->direction = "right";
    } else {
      response->direction = "forward";
    }

    RCLCPP_INFO(this->get_logger(), "Determined safest direction: %s",
                response->direction.c_str());
  }

  rclcpp::Service<GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
