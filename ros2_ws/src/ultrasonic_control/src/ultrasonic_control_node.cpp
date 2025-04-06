/*
© 2025 Falk-Richard Kallmer, Leuphana University Lüneburg
MIT License – see LICENSE file for details

This ROS2 node integrates ultrasonic sensor data to control a robot's movement along two axes ("x" and "y"). 
It processes sensor readings to enable/disable the robot or issue jog commands for position adjustments.
*/

#include "ultrasonic_control/ultrasonic_control_node.hpp"

using namespace std::chrono_literals;

namespace ultrasonic_control
{

// Constructor: Initializes the node, sets up clients, subscriptions, and callback groups.
UltrasonicControlNode::UltrasonicControlNode(const rclcpp::NodeOptions & node_options)
: Node("ultrasonic_control_node", node_options), is_robot_enabled_(false), last_robot_mode_(0), last_jog_mode_(""),
  last_enable_call_(this->get_clock()->now()), last_disable_call_(this->get_clock()->now()), active_axis_("")
{
  // Create a mutually exclusive callback group and executor
  this->callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  this->callback_group_executor_.add_callback_group(
    this->callback_group_, this->get_node_base_interface());

  // Initialize service clients for enabling/disabling the robot and moving in jog mode
  enable_robot_clnt_ = this->create_client<mg400_msgs::srv::EnableRobot>(
    "/mg400/enable_robot", rmw_qos_profile_default, callback_group_);
  disable_robot_clnt_ = this->create_client<mg400_msgs::srv::DisableRobot>(
    "/mg400/disable_robot", rmw_qos_profile_default, callback_group_);
  move_jog_clnt_ = this->create_client<mg400_msgs::srv::MoveJog>(
    "/mg400/move_jog", rmw_qos_profile_default, callback_group_);

  // Subscribe to ultrasonic sensor topics for "x" and "y" axes
  sub_x_ = create_subscription<sensor_msgs::msg::Range>(
    "/sensor/ultrasonic/x", 10,
    std::bind(&UltrasonicControlNode::distance_callback_x, this, std::placeholders::_1));

  sub_y_ = create_subscription<sensor_msgs::msg::Range>(
    "/sensor/ultrasonic/y", 10,
    std::bind(&UltrasonicControlNode::distance_callback_y, this, std::placeholders::_1));

  // Subscribe to the robot mode topic
  rm_sub_ = create_subscription<mg400_msgs::msg::RobotMode>(
    "/mg400/robot_mode", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&UltrasonicControlNode::robot_mode_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Node initialized.");
}

// Converts robot mode integer values to human-readable strings
std::string mode_to_string(uint64_t mode)
{
  switch (mode) {
    case 1: return "INIT";
    case 2: return "BRAKE_OPEN";
    case 4: return "DISABLED";
    case 5: return "ENABLE";
    case 6: return "BACKDRIVE";
    case 7: return "RUNNING";
    case 8: return "RECORDING";
    case 9: return "ERROR";
    case 10: return "PAUSE";
    case 11: return "JOG";
    case 12: return "INVALID";
    default: return "UNKNOWN";
  }
}

// Callback for robot mode changes; logs the new mode if it changes
void UltrasonicControlNode::robot_mode_callback(const mg400_msgs::msg::RobotMode::SharedPtr msg)
{
  if (!current_robot_mode_ || current_robot_mode_->robot_mode != msg->robot_mode) {
    current_robot_mode_ = msg;
    RCLCPP_INFO(this->get_logger(), "Robot mode changed: %s (%lu)", mode_to_string(msg->robot_mode).c_str(), msg->robot_mode);
  }
}

// Callback for "x" axis ultrasonic sensor data; triggers evaluation and action
void UltrasonicControlNode::distance_callback_x(const sensor_msgs::msg::Range::SharedPtr msg)
{
  distance_x_ = msg->range;
  if (active_axis_.empty() || active_axis_ == "x") {
    evaluate_and_act("x");
  }
}

// Callback for "y" axis ultrasonic sensor data; triggers evaluation and action
void UltrasonicControlNode::distance_callback_y(const sensor_msgs::msg::Range::SharedPtr msg)
{
  distance_y_ = msg->range;
  if (active_axis_.empty() || active_axis_ == "y") {
    evaluate_and_act("y");
  }
}

// Evaluates sensor data and determines whether to enable/disable the robot or issue jog commands
void UltrasonicControlNode::evaluate_and_act(const std::string & axis)
{
  auto now = get_clock()->now();
  float dist = (axis == "x") ? distance_x_ : distance_y_;
  auto mode = current_robot_mode_ ? current_robot_mode_->robot_mode : 0;

  // If the distance is greater than 0.20m, disable the robot if necessary
  if (dist > 0.20f) {
    if (last_jog_mode_ != "") {
      call_move_jog("");
    }
    if (mode == 5 && (now - last_disable_call_).seconds() > 2.0) {
      call_disable_robot();
      active_axis_ = "";
    }
    return;
  }

  // If the distance is within the range [0.08m, 0.12m], enable the robot if necessary
  if (dist >= 0.08f && dist <= 0.12f) {
    if (mode == 4 && (now - last_enable_call_).seconds() > 2.0) {
      active_axis_ = axis;
      call_enable_robot();
    }
    if (last_jog_mode_ != "") {
      call_move_jog("");
    }
    return;
  }

  // If the robot is enabled and the active axis matches, issue jog commands based on distance
  if (mode == 5 && active_axis_ == axis) {
    if (dist < 0.08f) {
      call_move_jog((axis == "x") ? "X-" : "j1+");
    } else if (dist > 0.12f) {
      call_move_jog((axis == "x") ? "X+" : "j1-");
    }
  }
}

// Sends a request to enable the robot
void UltrasonicControlNode::call_enable_robot()
{
  auto request = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();
  enable_robot_clnt_->async_send_request(request);
  last_enable_call_ = get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Enable robot requested.");
}

// Sends a request to disable the robot
void UltrasonicControlNode::call_disable_robot()
{
  auto request = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();
  disable_robot_clnt_->async_send_request(request);
  last_disable_call_ = get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Disable robot requested.");
}

// Sends a jog command to the robot
void UltrasonicControlNode::call_move_jog(const std::string & jog_mode)
{
  if (jog_mode == last_jog_mode_) return;

  auto request = std::make_shared<mg400_msgs::srv::MoveJog::Request>();
  request->jog.jog_mode = jog_mode;
  move_jog_clnt_->async_send_request(request);
  last_jog_mode_ = jog_mode;
  RCLCPP_INFO(this->get_logger(), "Jog command: %s", jog_mode.c_str());
}

}  // namespace ultrasonic_control

// Main function: Initializes the ROS2 node and starts spinning
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ultrasonic_control::UltrasonicControlNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
