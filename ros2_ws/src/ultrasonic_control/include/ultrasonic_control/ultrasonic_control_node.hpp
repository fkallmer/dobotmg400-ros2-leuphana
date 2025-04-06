/*
© 2025 Falk-Richard Kallmer, Leuphana University Lüneburg
MIT License – see LICENSE file for details
Contact: falk-richard.kallmer@stud.leuphana.de

This is the header file for the UltrasonicControlNode, which is responsible for controlling the robot based on ultrasonic sensor data.
*/

#ifndef ULTRASONIC_CONTROL__ULTRASONIC_CONTROL_NODE_HPP_
#define ULTRASONIC_CONTROL__ULTRASONIC_CONTROL_NODE_HPP_

// Include necessary ROS2 and custom message/service headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "mg400_msgs/srv/enable_robot.hpp"
#include "mg400_msgs/srv/disable_robot.hpp"
#include "mg400_msgs/srv/move_jog.hpp"
#include "mg400_msgs/msg/robot_mode.hpp"

namespace ultrasonic_control
{

// Define the UltrasonicControlNode class, inheriting from rclcpp::Node
class UltrasonicControlNode : public rclcpp::Node
{
public:
  // Constructor that accepts node options
  explicit UltrasonicControlNode(const rclcpp::NodeOptions & node_options);

private:
  // Callback functions for handling sensor and robot mode messages
  void distance_callback_x(const sensor_msgs::msg::Range::SharedPtr msg); // Callback for X-axis ultrasonic sensor
  void distance_callback_y(const sensor_msgs::msg::Range::SharedPtr msg); // Callback for Y-axis ultrasonic sensor
  void robot_mode_callback(const mg400_msgs::msg::RobotMode::SharedPtr msg); // Callback for robot mode updates
  void evaluate_and_act(const std::string & axis); // Evaluate sensor data and perform actions based on the active axis

  // Helper functions for making service calls
  void call_enable_robot(); // Enable the robot
  void call_disable_robot(); // Disable the robot
  void call_move_jog(const std::string & jog_mode); // Move the robot in jog mode

  // Service clients for interacting with the robot
  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_clnt_; // Client for enabling the robot
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_clnt_; // Client for disabling the robot
  rclcpp::Client<mg400_msgs::srv::MoveJog>::SharedPtr move_jog_clnt_; // Client for moving the robot in jog mode

  // Subscriptions for receiving sensor and robot mode data
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_x_; // Subscription for X-axis ultrasonic sensor
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_y_; // Subscription for Y-axis ultrasonic sensor
  rclcpp::Subscription<mg400_msgs::msg::RobotMode>::SharedPtr rm_sub_; // Subscription for robot mode updates

  // Variables to store sensor data and robot state
  float distance_x_{0.0f}; // Distance measured by the X-axis ultrasonic sensor
  float distance_y_{0.0f}; // Distance measured by the Y-axis ultrasonic sensor
  mg400_msgs::msg::RobotMode::SharedPtr current_robot_mode_; // Current robot mode
  bool is_robot_enabled_; // Flag indicating whether the robot is enabled
  uint64_t last_robot_mode_; // Last recorded robot mode
  std::string last_jog_mode_; // Last jog mode used
  std::string active_axis_; // Currently active axis for control

  // Timers for tracking service call timestamps
  rclcpp::Time last_enable_call_; // Timestamp of the last enable robot call
  rclcpp::Time last_disable_call_; // Timestamp of the last disable robot call

  // Callback execution management
  rclcpp::CallbackGroup::SharedPtr callback_group_; // Callback group for managing callbacks
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_; // Executor for the callback group
};

}  // namespace ultrasonic_control

#endif  // ULTRASONIC_CONTROL__ULTRASONIC_CONTROL_NODE_HPP_
