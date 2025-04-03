#ifndef ULTRASONIC_CONTROL__ULTRASONIC_CONTROL_NODE_HPP_
#define ULTRASONIC_CONTROL__ULTRASONIC_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "mg400_msgs/srv/enable_robot.hpp"
#include "mg400_msgs/srv/disable_robot.hpp"
#include "mg400_msgs/srv/move_jog.hpp"
#include "mg400_msgs/msg/robot_mode.hpp"

namespace ultrasonic_control
{

class UltrasonicControlNode : public rclcpp::Node
{
public:
  explicit UltrasonicControlNode(const rclcpp::NodeOptions & node_options);

private:
  // Callback functions
  void distance_callback_x(const sensor_msgs::msg::Range::SharedPtr msg);
  void distance_callback_y(const sensor_msgs::msg::Range::SharedPtr msg);
  void robot_mode_callback(const mg400_msgs::msg::RobotMode::SharedPtr msg);
  void evaluate_and_act(const std::string & axis);

  // Service call helpers
  void call_enable_robot();
  void call_disable_robot();
  void call_move_jog(const std::string & jog_mode);

  // Clients
  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::MoveJog>::SharedPtr move_jog_clnt_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_x_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_y_;
  rclcpp::Subscription<mg400_msgs::msg::RobotMode>::SharedPtr rm_sub_;

  // Values
  float distance_x_{0.0f};
  float distance_y_{0.0f};
  mg400_msgs::msg::RobotMode::SharedPtr current_robot_mode_;
  bool is_robot_enabled_;
  uint64_t last_robot_mode_;
  std::string last_jog_mode_;
  std::string active_axis_;


  // Timers
  rclcpp::Time last_enable_call_;
  rclcpp::Time last_disable_call_;

  // Callback execution
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace ultrasonic_control

#endif  // ULTRASONIC_CONTROL__ULTRASONIC_CONTROL_NODE_HPP_
