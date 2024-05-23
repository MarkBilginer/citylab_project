#include <cmath>
#include <custom_interfaces/action/go_to_pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = custom_interfaces::action::GoToPose;
  using GoalHandleGoToPoseAction =
      rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  GoToPose() : Node("go_to_pose_action_server_node") {
    // Create mutually exclusive callback group for action server
    this->action_server_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // Create reentrant callback group for odometry and velocity publishing
    this->reentrant_cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Initialize the action server options
    rcl_action_server_options_t action_server_options =
        rcl_action_server_get_default_options();
    // action_server_options.result_timeout.nanoseconds =
    // RCL_S_TO_NS(30); // Example of setting a specific option
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = reentrant_cb_group_;

    // Create an action server to handle incoming goal requests
    this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "/go_to_pose",
        std::bind(&GoToPose::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&GoToPose::handle_cancel, this, std::placeholders::_1),
        std::bind(&GoToPose::handle_accepted, this, std::placeholders::_1),
        action_server_options, action_server_cb_group_);
    if (!this->action_server_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create action server");
    }

    // Subscribe to odometry data
    this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&GoToPose::odom_callback, this, std::placeholders::_1),
        options1);

    // Publisher for sending velocity commands to the robot
    this->velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    if (!this->velocity_publisher_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create velocity publisher");
    }

    // Initialize desired and current position as Pose2D objects
    this->desired_pos_ = geometry_msgs::msg::Pose2D();
    this->current_pos_ = geometry_msgs::msg::Pose2D();

    // Create a timer to publish velocity commands periodically
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // Adjust the period as needed
        std::bind(&GoToPose::publish_velocity, this), reentrant_cb_group_);
    if (!this->timer_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create timer");
    }
  }

private:
  // Members to hold the action server, subscriber, publisher, and callback
  // groups
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::CallbackGroup::SharedPtr action_server_cb_group_;
  rclcpp::CallbackGroup::SharedPtr reentrant_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Variables to store the current and desired positions
  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Twist twist_msg_;

  std::mutex pos_mutex_;
  std::mutex twist_mutex_;

  // Thresholds for considering the goal as reached
  const double POSITION_TOLERANCE_ =
      0.1; // Tolerance for one significant digit for x and y
  const double ANGLE_TOLERANCE_ = 0.005; // 0.1 degrees tolerance for theta

  // Helper function to round to one significant digit
  double round_to_one_significant_digit(double value) {
    return std::floor(value * 10.0) / 10.0;
  }

  // Helper function to round to two significant digits
  double round_to_two_significant_digits(double value) {
    return std::floor(value * 100.0) / 100.0;
  }

  double round_to_four_significant_digits(double value) {
    return std::floor(value * 10000.0) / 10000.0;
  }

  // Conversion helper
  double degrees_to_radians(double degrees) { return degrees * M_PI / 180.0; }

  double radians_to_degrees(double radians) { return radians * 180.0 / M_PI; }

  // Handle incoming goal requests
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position x: %f, y: %f, theta: %f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;
    {
      std::lock_guard<std::mutex> lock(pos_mutex_);
      this->desired_pos_.x = goal->goal_pos.x;
      this->desired_pos_.y = goal->goal_pos.y;
      this->desired_pos_.theta = degrees_to_radians(goal->goal_pos.theta);
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle goal cancellation requests
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPoseAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handle the acceptance of a goal
  void
  handle_accepted(const std::shared_ptr<GoalHandleGoToPoseAction> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  // Execute the action (control loop)
  void execute(const std::shared_ptr<GoalHandleGoToPoseAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(30);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto result = std::make_shared<GoToPoseAction::Result>();

    // Control loop
    while (rclcpp::ok()) {
      double diff_x, diff_y, angle_to_goal, angle_diff;
      bool position_reached = false;
      bool orientation_reached = false;
      {
        std::lock_guard<std::mutex> lock(pos_mutex_);

        // Round current and desired positions to one significant digit
        double current_x_rounded =
            round_to_one_significant_digit(current_pos_.x);
        double current_y_rounded =
            round_to_one_significant_digit(current_pos_.y);
        double desired_x_rounded =
            round_to_one_significant_digit(desired_pos_.x);
        double desired_y_rounded =
            round_to_one_significant_digit(desired_pos_.y);
        double current_theta_rounded =
            round_to_four_significant_digits(current_pos_.theta);
        double desired_theta_rounded =
            round_to_four_significant_digits(desired_pos_.theta);

        // double desired_theta_rounded =
        // round_to_one_significant_digit(desired_pos_.theta);

        diff_x = desired_x_rounded - current_x_rounded;
        diff_y = desired_y_rounded - current_y_rounded;
        angle_to_goal = atan2(diff_y, diff_x);
        angle_diff = angle_to_goal - current_theta_rounded;

        // Normalize angle_diff to the range [-pi, pi]
        while (angle_diff > M_PI)
          angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI)
          angle_diff += 2.0 * M_PI;

        // Check if the position is reached
        position_reached = (fabs(diff_x) < POSITION_TOLERANCE_) &&
                           (fabs(diff_y) < POSITION_TOLERANCE_);

        // Check if the orientation is reached
        orientation_reached = (fabs(desired_theta_rounded -
                                    current_theta_rounded) < ANGLE_TOLERANCE_);
      }

      // Check if the goal is reached
      if (position_reached && orientation_reached) {

        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal reached");

        // Stop the robot
        {
          std::lock_guard<std::mutex> lock(twist_mutex_);
          twist_msg_.linear.x = 0.0;
          twist_msg_.angular.z = 0.0;
        }
        publish_velocity(); // Ensure the stop command is sent immediately

        return;
      }

      // Compute velocity commands
      {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        if (!position_reached) {
          // Move towards the goal position
          twist_msg_.linear.x =
              0.2 / 4; // Slow down the robot to improve accuracy
          twist_msg_.angular.z =
              fabs(angle_diff) < ANGLE_TOLERANCE_ ? 0.0 : angle_diff / 5;
        } else if (!orientation_reached) {
          // Rotate to the desired orientation
          twist_msg_.linear.x = 0.0;
          twist_msg_.angular.z = (desired_pos_.theta - current_pos_.theta) / 5;
        }
      }

      // Publish feedback
      {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        feedback->current_pos.x = current_pos_.x;
        feedback->current_pos.y = current_pos_.y;
        feedback->current_pos.theta = radians_to_degrees(current_pos_.theta);
      }
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
  }

  // Callback for processing odometry data
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(pos_mutex_);
      current_pos_.x = msg->pose.pose.position.x;
      current_pos_.y = msg->pose.pose.position.y;

      // Convert quaternion to Euler angles to get theta
      tf2::Quaternion q(
          msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      current_pos_.theta = yaw;
    }
  }

  void publish_velocity() {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    velocity_publisher_->publish(twist_msg_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
