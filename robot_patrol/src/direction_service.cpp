#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cmath>
#include <memory>

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
    const sensor_msgs::msg::LaserScan laser_scan = request->laser_data;
    const std::vector<float> ranges = laser_scan.ranges;
    const size_t num_points = ranges.size();

    if (num_points == 0) {
      RCLCPP_ERROR(this->get_logger(), "Received empty laser scan data.");
      return; // Early return to avoid further processing
    }

    if (num_points < 6) {
      RCLCPP_ERROR(this->get_logger(), "Insufficient laser scan data points.");
      return; // Early return to avoid further processing
    }

    const size_t middle_index =
        num_points / 2; // the front of the robot, pointing directly forward
    const size_t section_span =
        num_points / 6; // Each section represents 60 degrees

    // Define section indices for the front 180 degrees
    // Ensure all indices are of type 'size_t' and use static_cast to avoid type
    // mismatch warnings

    size_t front_start = static_cast<size_t>(middle_index - (section_span / 2));

    size_t front_end = static_cast<size_t>(middle_index + (section_span / 2));

    size_t left_start = front_end + 1;
    size_t left_end =
        static_cast<size_t>(middle_index + 3 * (section_span / 2));

    size_t right_start =
        static_cast<size_t>(middle_index - 3 * (section_span / 2));

    size_t right_end = front_start - 1;

    // Initialize total distances for left, front, and right sections as doubles
    double total_dist_sec_left = 0.0, total_dist_sec_front = 0.0,
           total_dist_sec_right = 0.0;
    bool left_safe = true, front_safe = true, right_safe = true;

    const double infinity_close_threshold =
        0.1; // Example threshold for very close distances
    const double infinity_far_threshold =
        2.0; // Example threshold for very far distances

    // Sum distances with infinite range handling
    auto add_distance = [&](size_t start, size_t end, double &total,
                            bool &safe) {
      for (size_t i = start; i <= end; ++i) {
        double distance =
            std::isinf(ranges[i]) ? laser_scan.range_max : ranges[i];
        if (distance < min_safe_distance_) {
          safe = false;
        }
        total += distance;
      }
    };

    add_distance(left_start, left_end, total_dist_sec_left, left_safe);
    add_distance(front_start, front_end, total_dist_sec_front, front_safe);
    add_distance(right_start, right_end, total_dist_sec_right, right_safe);

    RCLCPP_INFO(
        this->get_logger(), "Distance Totals - Left: %f, Front: %f, Right: %f",
        total_dist_sec_left, total_dist_sec_front, total_dist_sec_right);

    // Determine the safest direction based on the largest distance sum
    double safety_factor = 1.4; // Scaling factor to increase overall safety
    if (left_safe &&
        total_dist_sec_left * safety_factor > total_dist_sec_front &&
        total_dist_sec_left * safety_factor > total_dist_sec_right) {
      response->direction = "left";
    } else if (right_safe &&
               total_dist_sec_right * safety_factor > total_dist_sec_front &&
               total_dist_sec_right * safety_factor > total_dist_sec_left) {
      response->direction = "right";
    } else if (front_safe &&
               total_dist_sec_front * safety_factor > total_dist_sec_right &&
               total_dist_sec_front * safety_factor > total_dist_sec_left) {
      response->direction = "forward";
    } else {
      // Fallback mechanism: Choose the best among the unsafe sections
      RCLCPP_INFO(
          this->get_logger(),
          "All directions are unsafe. Choosing the best possible direction.");
      if (total_dist_sec_left * safety_factor > total_dist_sec_front &&
          total_dist_sec_left * safety_factor > total_dist_sec_right) {
        response->direction = "left";
      } else if (total_dist_sec_right * safety_factor > total_dist_sec_front &&
                 total_dist_sec_right * safety_factor > total_dist_sec_left) {
        response->direction = "right";
      } else if (total_dist_sec_front * safety_factor > total_dist_sec_right &&
                 total_dist_sec_front * safety_factor > total_dist_sec_left) {
        response->direction = "forward";
      }
    }
  }

  rclcpp::Service<GetDirection>::SharedPtr service_;
  const double min_safe_distance_ = 0.50; // 60 cm
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
